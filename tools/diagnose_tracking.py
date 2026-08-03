#!/usr/bin/env python3
"""Summarize synchronized Webots truth and ROS-free tracking diagnostics."""

import argparse
import itertools
import json
import math
import statistics
from pathlib import Path


def load_jsonl(path: Path) -> dict[int, dict]:
    records: dict[int, dict] = {}
    with path.open("r", encoding="utf-8") as stream:
        lines = stream.readlines()
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                records[int(record["seq"])] = record
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
                if line_number == len(lines) and not line.endswith("\n"):
                    continue
                raise ValueError(f"{path}:{line_number}: {error}") from error
    return records


def distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def detection_width(keypoints: list[list[float]]) -> float:
    top = distance(keypoints[1], keypoints[2])
    bottom = distance(keypoints[0], keypoints[3])
    return 0.5 * (top + bottom)


def detection_height(keypoints: list[list[float]]) -> float:
    left = distance(keypoints[0], keypoints[1])
    right = distance(keypoints[3], keypoints[2])
    return 0.5 * (left + right)


def point_mean(points: list[list[float]]) -> list[float]:
    return [statistics.mean(point[axis] for point in points)
            for axis in range(len(points[0]))]


def quaternion_rotate(quaternion: list[float], vector: list[float]) -> list[float] | None:
    matrix = quaternion_matrix(quaternion)
    if matrix is None or len(vector) != 3:
        return None
    return [
        sum(matrix[row * 3 + column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def ray_plane_intersection(
    pixel: list[float], truth_record: dict, armor: dict
) -> list[float] | None:
    intrinsics = truth_record.get("camera_intrinsics", [])
    orientation = truth_record.get("camera_orientation_wxyz", [])
    camera = truth_record.get("camera_position", [])
    center = armor.get("position", [])
    normal = armor.get("normal", [])
    if (len(pixel) != 2 or len(intrinsics) != 4 or len(camera) != 3 or
            len(center) != 3 or len(normal) != 3):
        return None
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 1e-9 or fy <= 1e-9:
        return None
    ray_camera = [(float(pixel[0]) - cx) / fx,
                  (float(pixel[1]) - cy) / fy, 1.0]
    ray_world = quaternion_rotate(orientation, ray_camera)
    if ray_world is None:
        return None
    denominator = sum(float(normal[i]) * ray_world[i] for i in range(3))
    if abs(denominator) <= 1e-9:
        return None
    scale = sum(float(normal[i]) * (float(center[i]) - float(camera[i]))
                for i in range(3)) / denominator
    if scale <= 0.0 or not math.isfinite(scale):
        return None
    return [float(camera[i]) + scale * ray_world[i] for i in range(3)]


def truth_armor_for_keypoints(
    keypoints: list[list[float]], truth_record: dict
) -> dict | None:
    if len(keypoints) != 4 or any(len(point) != 2 for point in keypoints):
        return None
    candidates = []
    detected_center = point_mean(keypoints)
    for armor in truth_record.get("armors", []):
        image_corners = armor.get("image_corners", [])
        world_corners = armor.get("world_corners", [])
        if (len(image_corners) != 4 or len(world_corners) != 4 or
                any(len(point) != 2 for point in image_corners) or
                any(len(point) != 3 for point in world_corners)):
            continue
        candidates.append((distance(detected_center, point_mean(image_corners)), armor))
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])[1]


def truth_projection_metrics(keypoints: list[list[float]], truth_record: dict) -> dict | None:
    armor = truth_armor_for_keypoints(keypoints, truth_record)
    if armor is None:
        return None
    detected_center = point_mean(keypoints)
    image_corners = armor["image_corners"]
    world_corners = armor["world_corners"]
    permutation = min(
        itertools.permutations(range(4)),
        key=lambda order: sum(
            distance(keypoints[index], image_corners[order[index]]) ** 2
            for index in range(4)
        ),
    )
    image_truth = [image_corners[index] for index in permutation]
    world_truth = [world_corners[index] for index in permutation]
    projected_width = detection_width(image_truth)
    projected_height = detection_height(image_truth)
    intersections = [ray_plane_intersection(point, truth_record, armor)
                     for point in keypoints]
    metrics = {
        "corner_pixel_errors": [
            distance(keypoints[index], image_truth[index]) for index in range(4)
        ],
        "center_pixel_error": distance(detected_center, point_mean(image_truth)),
        "projected_width_scale": detection_width(keypoints) / projected_width
        if projected_width > 1e-9 else None,
        "projected_height_scale": detection_height(keypoints) / projected_height
        if projected_height > 1e-9 else None,
        "backproject_corner_errors": [],
        "backproject_width_scale": None,
        "backproject_height_scale": None,
    }
    if all(point is not None for point in intersections):
        backprojected = [point for point in intersections if point is not None]
        metrics["backproject_corner_errors"] = [
            distance(backprojected[index], world_truth[index]) for index in range(4)
        ]
        truth_width = detection_width(world_truth)
        truth_height = detection_height(world_truth)
        if truth_width > 1e-9:
            metrics["backproject_width_scale"] = (
                detection_width(backprojected) / truth_width
            )
        if truth_height > 1e-9:
            metrics["backproject_height_scale"] = (
                detection_height(backprojected) / truth_height
            )
    return metrics


def projection_accumulator() -> dict[str, list[float] | int]:
    return {
        "matched_detections": 0,
        "corner_pixel_errors": [],
        "center_pixel_errors": [],
        "projected_width_scales": [],
        "projected_height_scales": [],
        "backproject_corner_errors": [],
        "backproject_width_scales": [],
        "backproject_height_scales": [],
    }


def accumulate_projection(accumulator: dict, metrics: dict | None) -> None:
    if metrics is None:
        return
    accumulator["matched_detections"] += 1
    accumulator["corner_pixel_errors"].extend(metrics["corner_pixel_errors"])
    accumulator["center_pixel_errors"].append(metrics["center_pixel_error"])
    for singular, plural in (
        ("projected_width_scale", "projected_width_scales"),
        ("projected_height_scale", "projected_height_scales"),
        ("backproject_width_scale", "backproject_width_scales"),
        ("backproject_height_scale", "backproject_height_scales"),
    ):
        if metrics[singular] is not None:
            accumulator[plural].append(metrics[singular])
    accumulator["backproject_corner_errors"].extend(
        metrics["backproject_corner_errors"]
    )


def projection_summary(accumulator: dict) -> dict:
    def distribution(values: list[float]) -> dict:
        return {
            "samples": len(values),
            "median": statistics.median(values) if values else None,
            "p10": percentile(values, 0.10),
            "p90": percentile(values, 0.90),
        }

    return {
        "matched_detections": accumulator["matched_detections"],
        "corner_pixel_error": distribution(accumulator["corner_pixel_errors"]),
        "center_pixel_error": distribution(accumulator["center_pixel_errors"]),
        "projected_width_scale": distribution(accumulator["projected_width_scales"]),
        "projected_height_scale": distribution(accumulator["projected_height_scales"]),
        "backproject_corner_error_m": distribution(
            accumulator["backproject_corner_errors"]
        ),
        "backproject_width_scale": distribution(
            accumulator["backproject_width_scales"]
        ),
        "backproject_height_scale": distribution(
            accumulator["backproject_height_scales"]
        ),
    }


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_periodic(value: float, period: float) -> float:
    return (value + 0.5 * period) % period - 0.5 * period


def axis_summary(errors: list[list[float]]) -> dict:
    labels = ("x", "y", "z")
    return {
        label: {
            "samples": len(errors[index]),
            "mean": statistics.mean(errors[index]) if errors[index] else None,
            "median_abs": statistics.median([abs(value) for value in errors[index]])
            if errors[index] else None,
            "p90_abs": percentile([abs(value) for value in errors[index]], 0.90),
        }
        for index, label in enumerate(labels)
    }


def pearson(left: list[float], right: list[float]) -> float | None:
    if len(left) != len(right) or len(left) < 2:
        return None
    left_mean = statistics.mean(left)
    right_mean = statistics.mean(right)
    numerator = sum(
        (a - left_mean) * (b - right_mean) for a, b in zip(left, right)
    )
    left_energy = sum((value - left_mean) ** 2 for value in left)
    right_energy = sum((value - right_mean) ** 2 for value in right)
    denominator = math.sqrt(left_energy * right_energy)
    return numerator / denominator if denominator > 1e-15 else None


def matrix_rpy(matrix: list[float]) -> tuple[float, float, float] | None:
    if len(matrix) != 9:
        return None
    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[6]))))
    roll = math.atan2(float(matrix[7]), float(matrix[8]))
    yaw = math.atan2(float(matrix[3]), float(matrix[0]))
    return roll, pitch, yaw


def quaternion_rpy(quaternion: list[float]) -> tuple[float, float, float] | None:
    if len(quaternion) != 4:
        return None
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return None
    w, x, y, z = (value / norm for value in (w, x, y, z))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quaternion_matrix(quaternion: list[float]) -> list[float] | None:
    if len(quaternion) != 4:
        return None
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return None
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return [
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
        2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y),
    ]


def matrix_multiply(left: list[float], right: list[float]) -> list[float]:
    return [
        sum(left[row * 3 + inner] * right[inner * 3 + column] for inner in range(3))
        for row in range(3)
        for column in range(3)
    ]


def matrix_rotation_error(left: list[float], right: list[float]) -> float:
    # trace(left^T * right) without constructing the transpose/product.
    trace = sum(left[row * 3 + column] * right[row * 3 + column]
                for row in range(3) for column in range(3))
    return math.acos(max(-1.0, min(1.0, 0.5 * (trace - 1.0))))


def align_panel_phase(
    estimate: list[float], truth: list[float]
) -> tuple[list[float], float] | None:
    if len(estimate) != 9 or len(truth) != 9:
        return None
    candidates = []
    for panel in range(4):
        phase = panel * math.pi / 2.0
        c, s = math.cos(phase), math.sin(phase)
        local_phase = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
        candidate = matrix_multiply(estimate, local_phase)
        candidates.append((matrix_rotation_error(candidate, truth), candidate))
    error, aligned = min(candidates, key=lambda value: value[0])
    return aligned, error


def quaternion_x_axis(quaternion: list[float]) -> list[float] | None:
    if len(quaternion) != 4:
        return None
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return None
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ]


def vector_angle(left: list[float], right: list[float]) -> float | None:
    if len(left) != 3 or len(right) != 3:
        return None
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm <= 1e-12 or right_norm <= 1e-12:
        return None
    cosine = sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    return math.acos(max(-1.0, min(1.0, cosine)))


def truth_center_control(truth_record: dict) -> list[float] | None:
    positions = [armor.get("position", []) for armor in truth_record.get("armors", [])]
    positions = [position for position in positions if len(position) == 3]
    if not positions:
        return None
    shooter = truth_record.get("shooter_position", [0.0, 0.0, 0.0])
    return [
        statistics.mean(position[index] for position in positions) - shooter[index]
        for index in range(3)
    ]


def build_truth_states(truth_records: list[dict]) -> dict[int, dict]:
    states: dict[int, dict] = {}
    previous: dict | None = None
    for record in truth_records:
        sequence = int(record["seq"])
        timestamp = float(record.get("sim_time_s", 0.0))
        center = truth_center_control(record)
        attitude = matrix_rpy(record.get("layout_orientation", []))
        if attitude is None:
            attitude = matrix_rpy(record.get("target_orientation", []))
        if attitude is None:
            armors = record.get("armors", [])
            normal = armors[0].get("normal", []) if armors else []
            attitude = (
                0.0,
                0.0,
                math.atan2(normal[1], normal[0]),
            ) if len(normal) >= 2 else (0.0, 0.0, 0.0)
        roll, pitch, yaw = attitude
        state = {
            "time": timestamp,
            "position": center,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
        if previous is not None and center is not None and previous["position"] is not None:
            dt = timestamp - previous["time"]
            if dt > 1e-6:
                # target_spinner drives Supervisor fields kinematically, for
                # which Webots getVelocity() may remain all-zero. Derivatives
                # of the recorded poses are therefore the authoritative truth.
                state["velocity"] = [
                    (center[index] - previous["position"][index]) / dt
                    for index in range(3)
                ]
                state["yaw_velocity"] = normalize_angle(yaw - previous["yaw"]) / dt
                state["angular_velocity"] = [0.0, 0.0, state["yaw_velocity"]]
                if "velocity" in previous:
                    state["acceleration"] = [
                        (state["velocity"][index] - previous["velocity"][index]) / dt
                        for index in range(3)
                    ]
                if "angular_velocity" in previous:
                    state["angular_acceleration"] = [
                        (state["angular_velocity"][index] - previous["angular_velocity"][index]) / dt
                        for index in range(3)
                    ]
                if "yaw_velocity" in previous:
                    state["yaw_acceleration"] = (
                        state["yaw_velocity"] - previous["yaw_velocity"]
                    ) / dt
                    state["angular_acceleration"] = [
                        0.0, 0.0, state["yaw_acceleration"]
                    ]
        states[sequence] = state
        previous = state
    return states


def longest_run(values: list[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def state_run_summary(states: list[int]) -> dict[str, dict[str, int]]:
    runs: dict[str, list[int]] = {}
    if states:
        current = states[0]
        length = 1
        for state in states[1:]:
            if state == current:
                length += 1
                continue
            runs.setdefault(str(current), []).append(length)
            current = state
            length = 1
        runs.setdefault(str(current), []).append(length)
    return {
        state: {"segments": len(lengths), "longest_frames": max(lengths)}
        for state, lengths in runs.items()
    }


def decision_bucket(reason: str) -> str:
    if reason.startswith("committed_single"):
        return "committed_single"
    if reason.startswith("committed_dual"):
        return "committed_dual"
    if reason.startswith("trial_rejected:"):
        detail = reason.removeprefix("trial_rejected:")
        return "trial_rejected:" + detail.split("=", 1)[0].split(":", 1)[0]
    if reason.startswith("commit_gate_fail"):
        return "commit_gate_fail"
    return reason or "unspecified"


def control_position(record: dict, truth_record: dict) -> list[float] | None:
    control = record.get("control_target", {})
    if not control.get("valid"):
        return None
    shooter = truth_record.get("shooter_position", [0.0, 0.0, 0.0])
    armors = truth_record.get("armors", [])
    tracks_center = control.get("tracks_center", False) or control.get("virtual_target", False)
    if tracks_center:
        position = truth_record.get("target_position", [])
    else:
        # Tracker panel zero has an arbitrary 90-degree phase, so its numeric
        # selected index cannot be mapped to Webots' fixed names. Compare the
        # predicted control point with the nearest physical truth panel.
        control_target = control.get("control_target_position", [])
        truth_positions = [armor.get("position", []) for armor in armors]
        truth_positions = [position for position in truth_positions if len(position) == 3]
        if len(control_target) != 3 or not truth_positions:
            return None
        position = min(
            truth_positions,
            key=lambda candidate: distance(
                control_target,
                [candidate[i] - shooter[i] for i in range(3)],
            ),
        )
    if len(position) != 3:
        return None
    return [position[i] - shooter[i] for i in range(3)]


def nearest_truth_by_time(truth_records: list[dict], target_time: float) -> dict | None:
    if not truth_records:
        return None
    return min(
        truth_records,
        key=lambda record: abs(float(record.get("sim_time_s", 0.0)) - target_time),
    )


def estimate_truth_structure(truth_record: dict) -> list[float] | None:
    armors = truth_record.get("armors", [])
    if len(armors) < 4:
        return None
    positions = [armor.get("position", []) for armor in armors[:4]]
    if any(len(position) != 3 for position in positions):
        return None
    pairings = ((0, 1, 2, 3), (0, 2, 1, 3), (0, 3, 1, 2))
    best = None
    for pairing in pairings:
        p0, p1, p2, p3 = pairing
        c0 = [(positions[p0][i] + positions[p1][i]) * 0.5 for i in range(3)]
        c1 = [(positions[p2][i] + positions[p3][i]) * 0.5 for i in range(3)]
        center_error = math.hypot(c0[0] - c1[0], c0[1] - c1[1])
        if best is None or center_error < best[0]:
            best = (center_error, pairing)
    _, (p0, p1, p2, p3) = best
    radius_a = 0.5 * distance(positions[p0][:2], positions[p1][:2])
    radius_b = 0.5 * distance(positions[p2][:2], positions[p3][:2])
    z_a = 0.5 * (positions[p0][2] + positions[p1][2])
    z_b = 0.5 * (positions[p2][2] + positions[p3][2])
    if z_a <= z_b:
        return [radius_a, radius_b, 0.5 * abs(z_a - z_b)]
    return [radius_b, radius_a, 0.5 * abs(z_a - z_b)]


def summarize(truth: dict[int, dict], tracking: dict[int, dict]) -> dict:
    common_sequences = sorted(truth.keys() & tracking.keys())
    truth_records = sorted(truth.values(), key=lambda record: record.get("sim_time_s", 0.0))
    depth_ratios: list[float] = []
    reprojection_errors: list[float] = []
    widths: list[float] = []
    direct_position_axis_errors: list[float] = []
    direct_yaw_errors: list[float] = []
    direct_surface_normal_errors: list[float] = []
    modes: dict[str, int] = {}
    states: dict[str, int] = {}
    mode_transitions = 0
    previous_mode = None
    current_center_errors: list[float] = []
    predicted_center_errors: list[float] = []
    control_target_errors: list[float] = []
    command_yaw_errors: list[float] = []
    prediction_times: list[float] = []
    structure_errors: list[float] = []
    structure_axis_errors = [[], [], []]
    direct_empty_flags: list[bool] = []
    ordered_states: list[int] = []
    update_valid = 0
    update_committed = 0
    update_decisions: dict[str, int] = {}
    estimated_yaw_rates: list[float] = []
    stationary_yaw_rates: list[float] = []
    yaw_rate_errors: list[float] = []
    yaw_phase_errors: list[float] = []
    yaw_sign_agreements: list[bool] = []
    state_position_axis_errors = [[], [], []]
    state_velocity_axis_errors = [[], [], []]
    state_acceleration_axis_errors = [[], [], []]
    state_angular_velocity_axis_errors = [[], [], []]
    state_angular_acceleration_axis_errors = [[], [], []]
    state_yaw_acceleration_errors: list[float] = []
    attitude_roll_errors: list[float] = []
    attitude_pitch_errors: list[float] = []
    attitude_yaw_errors: list[float] = []
    attitude_raw_roll_errors: list[float] = []
    attitude_raw_pitch_errors: list[float] = []
    attitude_up_errors: list[float] = []
    attitude_phase_equivalent_errors: list[float] = []
    estimated_z: list[float] = []
    estimated_vz: list[float] = []
    truth_yaw_rate_magnitudes: list[float] = []
    dza_estimates: list[float] = []
    dza_truths: list[float] = []
    tracked_armor_position_errors: list[float] = []
    tracked_armor_normal_errors: list[float] = []
    tracked_layout_yaw_rate_errors: list[float] = []
    tracked_layout_direction_agreements: list[bool] = []
    previous_tracked_layout: tuple[float, float] | None = None
    mpc_valid = 0
    mpc_success = 0
    mpc_fallback = 0
    mpc_iterations: list[float] = []
    mpc_active_constraints: list[float] = []
    mpc_costs: list[float] = []
    mpc_reference_yaw_rates: list[float] = []
    raw_projection = projection_accumulator()
    pnp_projection = projection_accumulator()
    pnp_position_axis_errors = [[], [], []]
    pnp_position_errors: list[float] = []
    pnp_radial_yaw_errors: list[float] = []
    pnp_surface_normal_errors: list[float] = []
    pnp_modes: dict[str, int] = {}

    truth_states = build_truth_states(truth_records)

    for sequence in common_sequences:
        truth_record = truth[sequence]
        tracking_record = tracking[sequence]
        mode = int(tracking_record.get("command_mode", 0))
        state = int(tracking_record.get("track_state", -1))
        modes[str(mode)] = modes.get(str(mode), 0) + 1
        states[str(state)] = states.get(str(state), 0) + 1
        ordered_states.append(state)
        if previous_mode is not None and previous_mode != mode:
            mode_transitions += 1
        previous_mode = mode

        direct_empty_flags.append(not tracking_record.get("direct_armors", []))
        tracker_update = tracking_record.get("tracker_update", {})
        if tracker_update.get("valid"):
            update_valid += 1
            committed = bool(tracker_update.get("committed"))
            update_committed += int(committed)
            bucket = decision_bucket(str(tracker_update.get("decision_reason", "")))
            update_decisions[bucket] = update_decisions.get(bucket, 0) + 1

        camera_position = truth_record.get("camera_position", [0.0, 0.0, 0.0])
        armor_distances = [
            distance(camera_position, armor["position"])
            for armor in truth_record.get("armors", [])
        ]
        shooter_position = truth_record.get("shooter_position", [0.0, 0.0, 0.0])
        truth_armors = truth_record.get("armors", [])
        truth_control_positions = [
            [value - shooter_position[index] for index, value in enumerate(armor["position"])]
            for armor in truth_armors
        ]
        truth_center = truth_center_control(truth_record)
        for direct in tracking_record.get("direct_armors", []):
            position = direct.get("position", [])
            if len(position) != 3 or not truth_control_positions:
                continue
            if "radial_yaw" in direct and truth_center is not None:
                truth_index = min(
                    range(len(truth_control_positions)),
                    key=lambda index: abs(
                        normalize_angle(
                            float(direct["radial_yaw"])
                            - math.atan2(
                                truth_control_positions[index][1] - truth_center[1],
                                truth_control_positions[index][0] - truth_center[0],
                            )
                        )
                    ),
                )
            else:
                truth_index = min(
                    range(len(truth_control_positions)),
                    key=lambda index: distance(position, truth_control_positions[index]),
                )
            truth_position = truth_control_positions[truth_index]
            direct_position_axis_errors.extend(
                position[index] - truth_position[index] for index in range(3)
            )
            normal = truth_armors[truth_index].get("normal", [])
            if len(normal) >= 2 and "radial_yaw" in direct:
                truth_radial_yaw = math.atan2(normal[1], normal[0])
                direct_yaw_errors.append(
                    normalize_angle(float(direct["radial_yaw"]) - truth_radial_yaw)
                )
            surface_normal = quaternion_x_axis(
                direct.get("surface_quaternion_wxyz", [])
            )
            surface_error = vector_angle(surface_normal or [], normal)
            if surface_error is not None:
                direct_surface_normal_errors.append(surface_error)
        tracked_armors = tracking_record.get("tracked_armors", [])
        for tracked_armor in tracked_armors:
            position = tracked_armor.get("position", [])
            if len(position) != 3 or not truth_control_positions:
                continue
            truth_index = min(
                range(len(truth_control_positions)),
                key=lambda index: distance(position, truth_control_positions[index]),
            )
            tracked_armor_position_errors.append(
                distance(position, truth_control_positions[truth_index])
            )
            normal_error = vector_angle(
                tracked_armor.get("normal", []),
                truth_armors[truth_index].get("normal", []),
            )
            if normal_error is not None:
                tracked_armor_normal_errors.append(normal_error)
        state_estimate = tracking_record.get("state_estimate", {})
        if tracked_armors and state_estimate.get("valid"):
            first_position = tracked_armors[0].get("position", [])
            center_position = state_estimate.get("position", [])
            if len(first_position) == 3 and len(center_position) == 3:
                layout_yaw = math.atan2(
                    float(first_position[1]) - float(center_position[1]),
                    float(first_position[0]) - float(center_position[0]),
                )
                timestamp = float(tracking_record.get("sim_time_s", 0.0))
                if previous_tracked_layout is not None:
                    previous_time, previous_yaw = previous_tracked_layout
                    dt = timestamp - previous_time
                    truth_yaw_rate = truth_states.get(sequence, {}).get("yaw_velocity")
                    if dt > 1e-6 and truth_yaw_rate is not None:
                        estimate_rate = normalize_angle(layout_yaw - previous_yaw) / dt
                        tracked_layout_yaw_rate_errors.append(
                            estimate_rate - float(truth_yaw_rate)
                        )
                        if abs(estimate_rate) > 0.2 and abs(truth_yaw_rate) > 0.2:
                            tracked_layout_direction_agreements.append(
                                estimate_rate * float(truth_yaw_rate) > 0.0
                            )
                previous_tracked_layout = (timestamp, layout_yaw)
        control = tracking_record.get("control_target", {})
        if control.get("valid"):
            estimated_yaw_rate = float(control.get("yaw_velocity_rad_s", 0.0))
            estimated_yaw_rates.append(estimated_yaw_rate)
            truth_state = truth_states.get(sequence, {})
            if "yaw_velocity" in truth_state:
                truth_yaw_rate = truth_state["yaw_velocity"]
                yaw_rate_errors.append(estimated_yaw_rate - truth_yaw_rate)
                if abs(truth_yaw_rate) > 0.2 and abs(estimated_yaw_rate) > 0.2:
                    yaw_sign_agreements.append(estimated_yaw_rate * truth_yaw_rate > 0.0)
                if abs(truth_yaw_rate) <= 0.05:
                    stationary_yaw_rates.append(estimated_yaw_rate)
        truth_state = truth_states.get(sequence, {})
        if state_estimate.get("valid"):
            for key, errors in (
                ("position", state_position_axis_errors),
                ("velocity", state_velocity_axis_errors),
                ("acceleration", state_acceleration_axis_errors),
            ):
                estimate_values = state_estimate.get(key, [])
                truth_values = truth_state.get(key, [])
                if len(estimate_values) == 3 and len(truth_values) == 3:
                    for index in range(3):
                        errors[index].append(
                            float(estimate_values[index]) - float(truth_values[index])
                        )
            if "yaw" in state_estimate and "yaw" in truth_state:
                yaw_phase_errors.append(
                    normalize_periodic(
                        float(state_estimate["yaw"]) - float(truth_state["yaw"]),
                        math.pi / 2.0,
                    )
                )
            estimate_attitude = quaternion_rpy(
                state_estimate.get("center_quaternion_wxyz", [])
            )
            if estimate_attitude is not None:
                attitude_raw_roll_errors.append(
                    normalize_angle(estimate_attitude[0] - truth_state.get("roll", 0.0))
                )
                attitude_raw_pitch_errors.append(
                    normalize_angle(estimate_attitude[1] - truth_state.get("pitch", 0.0))
                )
                attitude_yaw_errors.append(
                    normalize_periodic(
                        estimate_attitude[2] - truth_state.get("yaw", 0.0),
                        math.pi / 2.0,
                    )
                )
            estimate_matrix = quaternion_matrix(
                state_estimate.get("center_quaternion_wxyz", [])
            )
            truth_matrix = truth_record.get(
                "layout_orientation", truth_record.get("target_orientation", [])
            )
            if estimate_matrix is not None and len(truth_matrix) == 9:
                estimate_up = [estimate_matrix[2], estimate_matrix[5], estimate_matrix[8]]
                truth_up = [truth_matrix[2], truth_matrix[5], truth_matrix[8]]
                up_error = vector_angle(estimate_up, truth_up)
                if up_error is not None:
                    attitude_up_errors.append(up_error)
                phase_alignment = align_panel_phase(estimate_matrix, truth_matrix)
                if phase_alignment is not None:
                    aligned_matrix, orientation_error = phase_alignment
                    attitude_phase_equivalent_errors.append(orientation_error)
                    aligned_rpy = matrix_rpy(aligned_matrix)
                    truth_rpy = matrix_rpy(truth_matrix)
                    if aligned_rpy is not None and truth_rpy is not None:
                        attitude_roll_errors.append(
                            normalize_angle(aligned_rpy[0] - truth_rpy[0])
                        )
                        attitude_pitch_errors.append(
                            normalize_angle(aligned_rpy[1] - truth_rpy[1])
                        )
            for key, errors in (
                ("angular_velocity", state_angular_velocity_axis_errors),
                ("angular_acceleration", state_angular_acceleration_axis_errors),
            ):
                estimate_values = state_estimate.get(key, [])
                truth_values = truth_state.get(key, [])
                if len(estimate_values) == 3 and len(truth_values) == 3:
                    for index in range(3):
                        errors[index].append(
                            float(estimate_values[index]) - float(truth_values[index])
                        )
            if "yaw_acceleration" in state_estimate and "yaw_acceleration" in truth_state:
                state_yaw_acceleration_errors.append(
                    float(state_estimate["yaw_acceleration"])
                    - float(truth_state["yaw_acceleration"])
                )
            position = state_estimate.get("position", [])
            velocity = state_estimate.get("velocity", [])
            if len(position) == 3:
                estimated_z.append(float(position[2]))
            if len(velocity) == 3:
                estimated_vz.append(float(velocity[2]))
                truth_yaw_rate_magnitudes.append(
                    abs(float(truth_state.get("yaw_velocity", 0.0)))
                )
        mpc = tracking_record.get("mpc", {})
        if mpc.get("valid"):
            mpc_valid += 1
            mpc_success += int(bool(mpc.get("qp_success")))
            mpc_fallback += int(bool(mpc.get("fallback_used")))
            mpc_iterations.append(float(mpc.get("qp_iterations", 0)))
            mpc_active_constraints.append(float(mpc.get("active_set_size", 0)))
            cost = float(mpc.get("qp_cost", 0.0))
            if math.isfinite(cost):
                mpc_costs.append(cost)
            for reference in mpc.get("reference_states", []):
                if len(reference) == 4:
                    mpc_reference_yaw_rates.append(float(reference[2]))
        tracker_structure = tracking_record.get("tracker_structure", {})
        truth_structure = estimate_truth_structure(truth_record)
        if tracker_structure.get("valid") and truth_structure is not None:
            estimate = [
                float(tracker_structure.get("r1", 0.0)),
                float(tracker_structure.get("r2", 0.0)),
                float(tracker_structure.get("dza", 0.0)),
            ]
            # Panel zero is whichever plate initializes the tracker. Shifting
            # that phase by 90 degrees swaps radii and flips dza while
            # preserving the exact same physical four-panel layout.
            equivalent_truth = [
                truth_structure[1], truth_structure[0], -truth_structure[2]
            ]
            candidates = (
                [estimate[i] - truth_structure[i] for i in range(3)],
                [estimate[i] - equivalent_truth[i] for i in range(3)],
            )
            errors = min(
                candidates, key=lambda values: distance(values, [0.0, 0.0, 0.0])
            )
            structure_axis_errors[0].append(errors[0])
            structure_axis_errors[1].append(errors[1])
            structure_axis_errors[2].append(errors[2])
            structure_errors.append(distance(errors, [0.0, 0.0, 0.0]))
            dza_estimates.append(float(tracker_structure.get("dza", 0.0)))
            dza_truths.append(float(truth_structure[2]))
        current_center = control.get("current_center", [])
        truth_control_positions = [
            [armor["position"][i] - shooter_position[i] for i in range(3)]
            for armor in truth_record.get("armors", [])
        ]
        current_truth_center = [
            statistics.mean(position[i] for position in truth_control_positions)
            for i in range(3)
        ] if truth_control_positions else []
        if len(current_center) == 3 and control.get("valid"):
            current_center_errors.append(distance(current_center, current_truth_center))

        prediction_time = float(control.get("prediction_time_s", 0.0))
        if prediction_time >= 0.0 and control.get("valid"):
            prediction_times.append(prediction_time)
            future_truth = nearest_truth_by_time(
                truth_records,
                float(truth_record.get("sim_time_s", 0.0)) + prediction_time,
            )
            predicted_center = control.get("predicted_center", [])
            if future_truth and len(predicted_center) == 3:
                future_shooter = future_truth.get("shooter_position", [0.0, 0.0, 0.0])
                future_control_positions = [
                    [armor["position"][i] - future_shooter[i] for i in range(3)]
                    for armor in future_truth.get("armors", [])
                ]
                future_center = [
                    statistics.mean(position[i] for position in future_control_positions)
                    for i in range(3)
                ]
                predicted_center_errors.append(distance(predicted_center, future_center))
                target_position = control_position(tracking_record, future_truth)
                control_target = control.get("control_target_position", [])
                if target_position is not None and len(control_target) == 3:
                    control_target_errors.append(distance(control_target, target_position))
                command = tracking_record.get("command", {})
                if target_position is not None and len(target_position) == 3:
                    command_yaw = command.get("yaw_deg")
                    if command_yaw is not None:
                        target_yaw = math.degrees(math.atan2(target_position[1], target_position[0]))
                        command_yaw_errors.append(
                            math.degrees(normalize_angle(math.radians(float(command_yaw)) - math.radians(target_yaw)))
                        )
        for detection in tracking_record.get("detections", []):
            keypoints = detection.get("keypoints", [])
            if len(keypoints) == 4:
                widths.append(detection_width(keypoints))
                accumulate_projection(
                    raw_projection,
                    truth_projection_metrics(keypoints, truth_record),
                )
            pose_keypoints = detection.get("pose_keypoints", [])
            if len(pose_keypoints) == 4:
                accumulate_projection(
                    pnp_projection,
                    truth_projection_metrics(pose_keypoints, truth_record),
                )
            pose = detection.get("pose")
            if not pose or not armor_distances:
                continue
            translation = pose.get("translation", [])
            if len(translation) != 3:
                continue
            pnp_modes[str(int(pose.get("mode", -1)))] = (
                pnp_modes.get(str(int(pose.get("mode", -1))), 0) + 1
            )
            truth_armor = truth_armor_for_keypoints(keypoints, truth_record)
            camera_orientation = truth_record.get("camera_orientation_wxyz", [])
            world_offset = quaternion_rotate(camera_orientation, translation)
            if truth_armor is not None and world_offset is not None:
                truth_position = truth_armor.get("position", [])
                if len(camera_position) == 3 and len(truth_position) == 3:
                    pnp_world = [
                        float(camera_position[axis]) + world_offset[axis]
                        for axis in range(3)
                    ]
                    position_error = [
                        pnp_world[axis] - float(truth_position[axis])
                        for axis in range(3)
                    ]
                    for axis in range(3):
                        pnp_position_axis_errors[axis].append(position_error[axis])
                    pnp_position_errors.append(distance(pnp_world, truth_position))
                pnp_normal_camera = quaternion_rotate(
                    pose.get("rotation_wxyz", []), [1.0, 0.0, 0.0]
                )
                pnp_normal_world = quaternion_rotate(
                    camera_orientation, pnp_normal_camera or []
                )
                truth_normal = truth_armor.get("normal", [])
                if pnp_normal_world is not None and len(truth_normal) == 3:
                    # The PnP object X axis points into the visible plate;
                    # Pipeline::updateTracking adds pi to recover outward radial yaw.
                    outward = [-value for value in pnp_normal_world]
                    normal_error = vector_angle(outward, truth_normal)
                    if normal_error is not None:
                        pnp_surface_normal_errors.append(normal_error)
                    pnp_radial_yaw_errors.append(normalize_angle(
                        math.atan2(outward[1], outward[0]) -
                        math.atan2(float(truth_normal[1]), float(truth_normal[0]))
                    ))
            estimated_range = math.sqrt(sum(value * value for value in translation))
            closest_truth_range = min(armor_distances, key=lambda value: abs(value - estimated_range))
            if closest_truth_range > 1e-9:
                depth_ratios.append(estimated_range / closest_truth_range)
            reprojection_errors.append(float(pose.get("reprojection_error", 0.0)))

    return {
        "truth_frames": len(truth),
        "tracking_frames": len(tracking),
        "aligned_frames": len(common_sequences),
        "command_modes": modes,
        "track_states": states,
        "track_state_runs": state_run_summary(ordered_states),
        "command_mode_transitions": mode_transitions,
        "input_continuity": {
            "direct_empty_frames": sum(direct_empty_flags),
            "direct_empty_ratio": sum(direct_empty_flags) / len(direct_empty_flags)
            if direct_empty_flags else None,
            "longest_direct_empty_run_frames": longest_run(direct_empty_flags),
        },
        "tracker_updates": {
            "valid": update_valid,
            "committed": update_committed,
            "rejected": update_valid - update_committed,
            "decision_counts": update_decisions,
        },
        "yaw_rate_estimate_rad_s": {
            "samples": len(estimated_yaw_rates),
            "median_abs": statistics.median([abs(value) for value in estimated_yaw_rates])
            if estimated_yaw_rates else None,
            "p90_abs": percentile([abs(value) for value in estimated_yaw_rates], 0.90),
            "max_abs": max([abs(value) for value in estimated_yaw_rates], default=None),
            "abs_over_0_1_frames": sum(abs(value) > 0.1 for value in estimated_yaw_rates),
            "truth_error_p90_abs": percentile(
                [abs(value) for value in yaw_rate_errors], 0.90
            ),
            "truth_error_median": statistics.median(yaw_rate_errors)
            if yaw_rate_errors else None,
            "direction_agreement_ratio":
            sum(yaw_sign_agreements) / len(yaw_sign_agreements)
            if yaw_sign_agreements else None,
            "stationary_truth_samples": len(stationary_yaw_rates),
            "stationary_truth_p90_abs": percentile(
                [abs(value) for value in stationary_yaw_rates], 0.90
            ),
            "stationary_truth_max_abs": max(
                [abs(value) for value in stationary_yaw_rates], default=None
            ),
            "stationary_truth_abs_over_0_1_frames": sum(
                abs(value) > 0.1 for value in stationary_yaw_rates
            ),
        },
        "depth_ratio_estimated_over_nearest_truth": {
            "samples": len(depth_ratios),
            "median": statistics.median(depth_ratios) if depth_ratios else None,
            "p10": percentile(depth_ratios, 0.10),
            "p90": percentile(depth_ratios, 0.90),
        },
        "keypoint_width_pixels": {
            "samples": len(widths),
            "median": statistics.median(widths) if widths else None,
            "p10": percentile(widths, 0.10),
            "p90": percentile(widths, 0.90),
        },
        "reprojection_error_pixels": {
            "samples": len(reprojection_errors),
            "median": statistics.median(reprojection_errors) if reprojection_errors else None,
            "p90": percentile(reprojection_errors, 0.90),
        },
        "truth_projection_error": {
            "raw_keypoints": projection_summary(raw_projection),
            "pnp_input_keypoints": projection_summary(pnp_projection),
        },
        "vision_pnp_error": {
            "modes": pnp_modes,
            "position_axis_m": axis_summary(pnp_position_axis_errors),
            "position_m": {
                "samples": len(pnp_position_errors),
                "median": statistics.median(pnp_position_errors)
                if pnp_position_errors else None,
                "p90": percentile(pnp_position_errors, 0.90),
            },
            "radial_yaw_error_rad": {
                "samples": len(pnp_radial_yaw_errors),
                "median": statistics.median(pnp_radial_yaw_errors)
                if pnp_radial_yaw_errors else None,
                "median_abs": statistics.median(
                    [abs(value) for value in pnp_radial_yaw_errors]
                ) if pnp_radial_yaw_errors else None,
                "p90_abs": percentile(
                    [abs(value) for value in pnp_radial_yaw_errors], 0.90
                ),
            },
            "surface_normal_error_rad": {
                "samples": len(pnp_surface_normal_errors),
                "median": statistics.median(pnp_surface_normal_errors)
                if pnp_surface_normal_errors else None,
                "p90": percentile(pnp_surface_normal_errors, 0.90),
            },
        },
        "direct_position_axis_error_m": {
            "samples": len(direct_position_axis_errors),
            "mean": statistics.mean(direct_position_axis_errors)
            if direct_position_axis_errors else None,
            "stddev": statistics.pstdev(direct_position_axis_errors)
            if direct_position_axis_errors else None,
            "p90_abs": percentile([abs(value) for value in direct_position_axis_errors], 0.90),
        },
        "direct_yaw_error_rad": {
            "samples": len(direct_yaw_errors),
            "mean": statistics.mean(direct_yaw_errors) if direct_yaw_errors else None,
            "stddev": statistics.pstdev(direct_yaw_errors) if direct_yaw_errors else None,
            "p90_abs": percentile([abs(value) for value in direct_yaw_errors], 0.90),
        },
        "direct_surface_normal_error_rad": {
            "samples": len(direct_surface_normal_errors),
            "median": statistics.median(direct_surface_normal_errors)
            if direct_surface_normal_errors else None,
            "p90": percentile(direct_surface_normal_errors, 0.90),
        },
        "tracked_armor_error": {
            "position_m": {
                "samples": len(tracked_armor_position_errors),
                "median": statistics.median(tracked_armor_position_errors)
                if tracked_armor_position_errors else None,
                "p90": percentile(tracked_armor_position_errors, 0.90),
            },
            "normal_angle_rad": {
                "samples": len(tracked_armor_normal_errors),
                "median": statistics.median(tracked_armor_normal_errors)
                if tracked_armor_normal_errors else None,
                "p90": percentile(tracked_armor_normal_errors, 0.90),
            },
            "layout_yaw_rate_error_rad_s": {
                "samples": len(tracked_layout_yaw_rate_errors),
                "median": statistics.median(tracked_layout_yaw_rate_errors)
                if tracked_layout_yaw_rate_errors else None,
                "p90_abs": percentile(
                    [abs(value) for value in tracked_layout_yaw_rate_errors], 0.90
                ),
                "direction_agreement_ratio": sum(tracked_layout_direction_agreements)
                / len(tracked_layout_direction_agreements)
                if tracked_layout_direction_agreements else None,
            },
        },
        "state_estimate_error": {
            "position_m": axis_summary(state_position_axis_errors),
            "velocity_mps": axis_summary(state_velocity_axis_errors),
            "acceleration_mps2": axis_summary(state_acceleration_axis_errors),
            "angular_velocity_rad_s": axis_summary(state_angular_velocity_axis_errors),
            "angular_acceleration_rad_s2": axis_summary(
                state_angular_acceleration_axis_errors
            ),
            "yaw_acceleration_error_rad_s2": {
                "samples": len(state_yaw_acceleration_errors),
                "median": statistics.median(state_yaw_acceleration_errors)
                if state_yaw_acceleration_errors else None,
                "p90_abs": percentile(
                    [abs(value) for value in state_yaw_acceleration_errors], 0.90
                ),
            },
            "yaw_phase_error_rad_mod_pi_over_2": {
                "samples": len(yaw_phase_errors),
                "median_abs": statistics.median([abs(value) for value in yaw_phase_errors])
                if yaw_phase_errors else None,
                "p90_abs": percentile([abs(value) for value in yaw_phase_errors], 0.90),
            },
            "roll_error_rad": {
                "samples": len(attitude_roll_errors),
                "median_abs": statistics.median([abs(value) for value in attitude_roll_errors])
                if attitude_roll_errors else None,
                "p90_abs": percentile([abs(value) for value in attitude_roll_errors], 0.90),
            },
            "pitch_error_rad": {
                "samples": len(attitude_pitch_errors),
                "median_abs": statistics.median([abs(value) for value in attitude_pitch_errors])
                if attitude_pitch_errors else None,
                "p90_abs": percentile([abs(value) for value in attitude_pitch_errors], 0.90),
            },
            "raw_phase_dependent_roll_error_rad": {
                "samples": len(attitude_raw_roll_errors),
                "median_abs": statistics.median(
                    [abs(value) for value in attitude_raw_roll_errors]
                ) if attitude_raw_roll_errors else None,
                "p90_abs": percentile(
                    [abs(value) for value in attitude_raw_roll_errors], 0.90
                ),
            },
            "raw_phase_dependent_pitch_error_rad": {
                "samples": len(attitude_raw_pitch_errors),
                "median_abs": statistics.median(
                    [abs(value) for value in attitude_raw_pitch_errors]
                ) if attitude_raw_pitch_errors else None,
                "p90_abs": percentile(
                    [abs(value) for value in attitude_raw_pitch_errors], 0.90
                ),
            },
            "up_vector_angle_error_rad": {
                "samples": len(attitude_up_errors),
                "median": statistics.median(attitude_up_errors)
                if attitude_up_errors else None,
                "p90": percentile(attitude_up_errors, 0.90),
            },
            "orientation_error_rad_mod_panel_phase": {
                "samples": len(attitude_phase_equivalent_errors),
                "median": statistics.median(attitude_phase_equivalent_errors)
                if attitude_phase_equivalent_errors else None,
                "p90": percentile(attitude_phase_equivalent_errors, 0.90),
            },
            "quaternion_yaw_error_rad_mod_pi_over_2": {
                "samples": len(attitude_yaw_errors),
                "median_abs": statistics.median([abs(value) for value in attitude_yaw_errors])
                if attitude_yaw_errors else None,
                "p90_abs": percentile([abs(value) for value in attitude_yaw_errors], 0.90),
            },
            "vertical_dynamics": {
                "z_peak_to_peak_m": max(estimated_z) - min(estimated_z)
                if estimated_z else None,
                "vz_p90_abs_mps": percentile([abs(value) for value in estimated_vz], 0.90),
                "abs_vz_vs_abs_truth_yaw_rate_correlation": pearson(
                    [abs(value) for value in estimated_vz], truth_yaw_rate_magnitudes
                ),
                "z_final_quarter_peak_to_peak_m": (
                    max(estimated_z[(3 * len(estimated_z)) // 4 :])
                    - min(estimated_z[(3 * len(estimated_z)) // 4 :])
                ) if estimated_z else None,
            },
        },
        "mpc": {
            "samples": mpc_valid,
            "success_ratio": mpc_success / mpc_valid if mpc_valid else None,
            "fallback_ratio": mpc_fallback / mpc_valid if mpc_valid else None,
            "iterations_median": statistics.median(mpc_iterations)
            if mpc_iterations else None,
            "active_constraints_median": statistics.median(mpc_active_constraints)
            if mpc_active_constraints else None,
            "cost_median": statistics.median(mpc_costs) if mpc_costs else None,
            "reference_yaw_rate_p90_abs": percentile(
                [abs(value) for value in mpc_reference_yaw_rates], 0.90
            ),
        },
        "current_center_error_m": {
            "samples": len(current_center_errors),
            "median": statistics.median(current_center_errors) if current_center_errors else None,
            "p90": percentile(current_center_errors, 0.90),
        },
        "predicted_center_error_at_prediction_time_m": {
            "samples": len(predicted_center_errors),
            "median": statistics.median(predicted_center_errors) if predicted_center_errors else None,
            "p90": percentile(predicted_center_errors, 0.90),
        },
        "control_target_error_m": {
            "samples": len(control_target_errors),
            "median": statistics.median(control_target_errors) if control_target_errors else None,
            "p90": percentile(control_target_errors, 0.90),
        },
        "prediction_time_s": {
            "samples": len(prediction_times),
            "median": statistics.median(prediction_times) if prediction_times else None,
            "p90": percentile(prediction_times, 0.90),
        },
        "command_yaw_error_deg": {
            "samples": len(command_yaw_errors),
            "median_abs": statistics.median([abs(value) for value in command_yaw_errors])
            if command_yaw_errors else None,
            "p90_abs": percentile([abs(value) for value in command_yaw_errors], 0.90),
        },
        "tracker_structure_error_m": {
            "samples": len(structure_errors),
            "median": statistics.median(structure_errors) if structure_errors else None,
            "p90": percentile(structure_errors, 0.90),
            "r1_p90_abs": percentile([abs(value) for value in structure_axis_errors[0]], 0.90),
            "r2_p90_abs": percentile([abs(value) for value in structure_axis_errors[1]], 0.90),
            "dza_p90_abs": percentile([abs(value) for value in structure_axis_errors[2]], 0.90),
            "dza_truth_median": statistics.median(dza_truths) if dza_truths else None,
            "dza_estimate_median": statistics.median(dza_estimates)
            if dza_estimates else None,
            "dza_estimate_final_quarter_median": statistics.median(
                dza_estimates[(3 * len(dza_estimates)) // 4 :]
            ) if dza_estimates else None,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge-dir", default="/tmp/hfut_auto_aim_webots")
    parser.add_argument("--truth", default="")
    parser.add_argument("--tracking", default="")
    parser.add_argument("--output", default="")
    arguments = parser.parse_args()
    bridge_directory = Path(arguments.bridge_dir)
    truth_path = Path(arguments.truth) if arguments.truth else bridge_directory / "target_truth.jsonl"
    tracking_path = (
        Path(arguments.tracking)
        if arguments.tracking
        else bridge_directory / "tracking_diagnostics.jsonl"
    )
    truth = load_jsonl(truth_path)
    tracking = load_jsonl(tracking_path)
    summary = summarize(truth, tracking)
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if arguments.output:
        Path(arguments.output).write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
