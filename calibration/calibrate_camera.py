#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""使用棋盘格图片生成 camera_info.yaml。"""

from __future__ import annotations

import argparse
import glob
import math
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print(f"[ERROR] 缺少 OpenCV Python 或 numpy：{exc}")
    print("        Ubuntu 可尝试：python3 -m pip install opencv-python numpy")
    sys.exit(2)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="用棋盘格图片标定相机内参。")
    parser.add_argument("--images", default="calibration/images/*.png", help="图片通配符。")
    parser.add_argument("--pattern-cols", type=int, default=9, help="棋盘格内角点列数。")
    parser.add_argument("--pattern-rows", type=int, default=6, help="棋盘格内角点行数。")
    parser.add_argument("--square-size", type=float, default=0.025, help="单个棋盘格边长，单位米。")
    parser.add_argument("--output", type=Path, default=Path("configs/camera_info.yaml"))
    parser.add_argument("--show", action="store_true", help="显示角点检测结果。")
    return parser.parse_args(argv)


def image_paths(pattern: str) -> list[Path]:
    paths = [Path(path) for path in glob.glob(pattern)]
    return sorted(path for path in paths if path.is_file())


def reprojection_error(objpoints: list[np.ndarray], imgpoints: list[np.ndarray],
                       rvecs: list[np.ndarray], tvecs: list[np.ndarray],
                       camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> float:
    total_error = 0.0
    total_points = 0
    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        error = cv2.norm(imgp, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(objp)
    return math.sqrt(total_error / max(total_points, 1))


def write_camera_info(path: Path, image_size: tuple[int, int], camera_matrix: np.ndarray,
                      dist_coeffs: np.ndarray, error: float) -> None:
    width, height = image_size
    d = dist_coeffs.reshape(-1).tolist()
    if len(d) < 5:
        d += [0.0] * (5 - len(d))
    d = d[:5]
    k = camera_matrix.reshape(-1).tolist()
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# 由 calibration/calibrate_camera.py 生成。\n")
        handle.write(f"# 平均重投影误差: {error:.6f} px\n")
        handle.write(f"image_width: {width}\n")
        handle.write(f"image_height: {height}\n")
        handle.write("camera_name: hfut_hik_camera\n")
        handle.write("camera_matrix:\n")
        handle.write("  rows: 3\n  cols: 3\n")
        handle.write("  data: [" + ", ".join(f"{value:.10g}" for value in k) + "]\n")
        handle.write("distortion_model: plumb_bob\n")
        handle.write("distortion_coefficients:\n")
        handle.write("  rows: 1\n  cols: 5\n")
        handle.write("  data: [" + ", ".join(f"{value:.10g}" for value in d) + "]\n")
        handle.write("rectification_matrix:\n")
        handle.write("  rows: 3\n  cols: 3\n")
        handle.write("  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
        handle.write("projection_matrix:\n")
        handle.write("  rows: 3\n  cols: 4\n")
        handle.write("  data: [" + ", ".join(f"{value:.10g}" for value in p) + "]\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.pattern_cols <= 0 or args.pattern_rows <= 0 or args.square_size <= 0.0:
        raise SystemExit("棋盘格内角点数量和 square-size 必须大于 0")

    paths = image_paths(args.images)
    if not paths:
        raise SystemExit(f"没有找到图片：{args.images}")

    pattern_size = (args.pattern_cols, args.pattern_rows)
    objp = np.zeros((args.pattern_cols * args.pattern_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.pattern_cols, 0:args.pattern_rows].T.reshape(-1, 2)
    objp *= float(args.square_size)

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"[WARN] 读取失败：{path}")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            print(f"[WARN] 尺寸不一致，跳过：{path}")
            continue

        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            print(f"[WARN] 未识别棋盘格：{path}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        print(f"[OK] {path}")

        if args.show:
            cv2.drawChessboardCorners(image, pattern_size, corners, found)
            cv2.imshow("calibrate_camera", image)
            key = cv2.waitKey(100)
            if key in (27, ord("q"), ord("Q")):
                break

    if args.show:
        cv2.destroyAllWindows()
    if image_size is None or len(objpoints) < 8:
        raise SystemExit(f"有效标定图片过少：{len(objpoints)}，建议至少 8-12 张")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )
    error = reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs)
    write_camera_info(args.output, image_size, camera_matrix, dist_coeffs, error)
    print(f"[OK] 输出：{args.output}")
    print(f"[OK] OpenCV RMS={rms:.6f}，平均重投影误差={error:.6f}px，有效图片={len(objpoints)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
