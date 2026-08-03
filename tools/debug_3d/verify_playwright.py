#!/usr/bin/env python3
import argparse
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright


def canvas_statistics(path):
    image = Image.open(path).convert("RGB")
    pixels = list(image.getdata())[::64]
    non_dark = sum(max(pixel) > 30 for pixel in pixels)
    unique = len(set(pixels))
    return {
        "width": image.width,
        "height": image.height,
        "sampled": len(pixels),
        "non_dark_ratio": non_dark / max(len(pixels), 1),
        "unique_colors": unique,
    }


def check_layout(page, mobile=False):
    rectangles = page.evaluate(
        """() => {
          const rect = (selector) => {
            const value = document.querySelector(selector).getBoundingClientRect();
            return {top: value.top, right: value.right, bottom: value.bottom,
                    left: value.left, width: value.width, height: value.height};
          };
          return {header: rect('.topbar'), inspector: rect('#inspector'),
                  timeline: rect('.timeline-bar'), canvas: rect('#scene canvas')};
        }"""
    )
    assert rectangles["header"]["bottom"] <= rectangles["timeline"]["top"]
    assert rectangles["canvas"]["width"] > 300
    assert rectangles["canvas"]["height"] > 300
    if mobile:
        page.click("#panel-button")
        page.wait_for_timeout(220)
        inspector = page.locator("#inspector").bounding_box()
        header = page.locator(".topbar").bounding_box()
        timeline = page.locator(".timeline-bar").bounding_box()
        assert inspector["x"] >= 0
        assert inspector["y"] >= header["y"] + header["height"] - 1
        assert inspector["y"] + inspector["height"] <= timeline["y"] + 1
    else:
        assert rectangles["inspector"]["top"] >= rectangles["header"]["bottom"] - 1
        assert rectangles["inspector"]["bottom"] <= rectangles["timeline"]["top"] + 1


def verify_view(browser, url, viewport, prefix, mobile=False):
    page = browser.new_page(viewport=viewport, device_scale_factor=1)
    errors = []
    page.on("console", lambda message: errors.append(message.text) if message.type == "error" else None)
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(url, wait_until="networkidle")
    page.wait_for_selector("#loading.hidden", state="attached", timeout=15000)
    page.wait_for_function("document.querySelector('#dataset-name').textContent.includes('frames')")
    page.wait_for_timeout(350)
    assert not errors, f"browser errors: {errors}"

    check_layout(page, mobile=mobile)
    page.locator("#timeline").evaluate(
        """element => { element.value = Math.min(400, Number(element.max));
                          element.dispatchEvent(new Event('input', {bubbles: true})); }"""
    )
    page.wait_for_timeout(180)
    assert page.locator("#value-position").inner_text() != "-"
    before = int(page.locator("#timeline").input_value())
    page.click("#play-button")
    page.wait_for_timeout(220)
    after = int(page.locator("#timeline").input_value())
    assert after > before, f"timeline did not advance: {before} -> {after}"
    page.click("#play-button")

    canvas = page.locator("#scene canvas")
    before_drag = canvas.screenshot()
    bounds = canvas.bounding_box()
    start_x = bounds["x"] + bounds["width"] * 0.36
    start_y = bounds["y"] + bounds["height"] * 0.48
    page.mouse.move(start_x, start_y)
    page.mouse.down()
    page.mouse.move(start_x + 110, start_y + 32, steps=8)
    page.mouse.up()
    page.wait_for_timeout(220)
    after_drag = canvas.screenshot()
    assert before_drag != after_drag, "orbit drag did not change the canvas"

    canvas_path = Path(f"/tmp/{prefix}_canvas.png")
    screenshot_path = Path(f"/tmp/{prefix}.png")
    canvas.screenshot(path=str(canvas_path))
    page.screenshot(path=str(screenshot_path), full_page=True)
    stats = canvas_statistics(canvas_path)
    assert stats["non_dark_ratio"] > 0.015, stats
    assert stats["unique_colors"] > 80, stats
    page.close()
    return stats, screenshot_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--browser", help="optional Chrome/Chromium executable path")
    args = parser.parse_args()
    with sync_playwright() as playwright:
        launch_options = {
            "headless": True,
            "args": ["--use-gl=angle", "--use-angle=swiftshader"],
        }
        if args.browser:
            launch_options["executable_path"] = args.browser
        browser = playwright.chromium.launch(**launch_options)
        desktop = verify_view(
            browser, args.url, {"width": 1440, "height": 900},
            "hfut_debug_3d_desktop",
        )
        mobile = verify_view(
            browser, args.url, {"width": 390, "height": 844},
            "hfut_debug_3d_mobile", mobile=True,
        )
        browser.close()
    print(f"desktop canvas: {desktop[0]}")
    print(f"mobile canvas: {mobile[0]}")
    print(f"screenshots: {desktop[1]}, {mobile[1]}")


if __name__ == "__main__":
    main()
