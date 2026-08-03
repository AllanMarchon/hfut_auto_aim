#!/usr/bin/env python3
"""outpost 闭环残差分解:tracker 中心/vyaw 误差 + 弹道/预测/选板/开火分解。

用法: analyze_outpost_run.py <diagnostics.jsonl> <target_truth.jsonl>
"""
import json
import math
import sys

G = 9.8
V = 22.5
K_DRAG = 0.0  # 二次阻力系数(a=-k|v|v)，hero 传 0.0175
SHOOTER = (0.0, 0.0, 0.405)


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[min(len(s) - 1, max(0, int(round((len(s) - 1) * q))))]


def solve_pitch(dx, dy, dz, v=None, g=None):
    """纯重力弹道:返回 (pitch_rad, flight_time_s),无解返回 None。
    r² + (dz + g·s/2)² = v²·s, s = t² —— 关于 s 的二次方程,取小根(低伸弹道)。"""
    if v is None:
        v = V
    if g is None:
        g = G
    r = math.hypot(dx, dy)
    A = g * g / 4.0
    B = g * dz - v * v
    C = r * r + dz * dz
    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    # 两个根中较小的 s(低伸弹道)
    s = (-B - sq) / (2.0 * A)
    if s <= 0.0:
        s = (-B + sq) / (2.0 * A)
    if s <= 0.0:
        return None
    t = math.sqrt(s)
    pitch = math.atan2(dz + 0.5 * g * s, r)
    return pitch, t


def yaw_to(dx, dy):
    return math.atan2(dy, dx)


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def truth_yaw(ori):
    # 3x3 行优先旋转矩阵
    return math.atan2(ori[3], ori[0])


def main(diag_path, truth_path):
    truth = []
    with open(truth_path) as f:
        for line in f:
            try:
                truth.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    truth.sort(key=lambda d: d["sim_time_s"])
    times = [d["sim_time_s"] for d in truth]

    def truth_at(t):
        import bisect
        i = bisect.bisect_left(times, t)
        if i >= len(truth):
            i = len(truth) - 1
        if i > 0 and abs(times[i - 1] - t) < abs(times[i] - t):
            i -= 1
        return truth[i] if abs(times[i] - t) < 0.06 else None

    # 真值 vyaw 数值微分
    def truth_vyaw(idx):
        if idx <= 0 or idx >= len(truth) - 1:
            return float("nan")
        dt = truth[idx + 1]["sim_time_s"] - truth[idx - 1]["sim_time_s"]
        if dt <= 0:
            return float("nan")
        return ang_diff(truth_yaw(truth[idx + 1]["target_orientation"]),
                        truth_yaw(truth[idx - 1]["target_orientation"])) / dt

    import bisect
    res = {k: [] for k in (
        "center_err", "center_err_xy", "vyaw_err",
        "err_solver_yaw", "err_solver_pitch",
        "err_pred_yaw", "err_pred_pitch",
        "err_total_yaw", "err_total_pitch",
        "gimbal_lag_yaw", "gimbal_lag_pitch",
    )}
    fire = {k: [] for k in ("err_fire_yaw", "err_fire_pitch", "miss_dist")}
    n_frames = n_fire = 0

    with open(diag_path) as f:
        for line in f:
            d = json.loads(line)
            if d.get("track_state", -1) < 2:
                continue
            cmd = d.get("command") or {}
            ct = d.get("control_target") or {}
            est = d.get("state_estimate") or {}
            if not (cmd.get("distance_m") and ct.get("valid") and est.get("valid")):
                continue
            t = d["sim_time_s"]
            tr = truth_at(t)
            if tr is None:
                continue
            n_frames += 1

            # tracker 中心误差
            ec = [est["position"][i] - tr["target_position"][i] for i in range(3)]
            res["center_err"].append(math.sqrt(sum(e * e for e in ec)))
            res["center_err_xy"].append(math.hypot(ec[0], ec[1]))
            idx = bisect.bisect_left(times, t)
            tv = truth_vyaw(idx)
            if not math.isnan(tv):
                res["vyaw_err"].append(est["yaw_velocity"] - tv)

            # 选中板: tracked_armors[selected_index] -> 最近真值板
            si = ct.get("selected_index", 0)
            armors = d.get("tracked_armors") or []
            if si >= len(armors):
                continue
            sel_pos = armors[si]["position"]
            bi = min(range(len(tr["armors"])),
                     key=lambda i: sum((tr["armors"][i]["position"][j] - sel_pos[j]) ** 2
                                       for j in range(3)))
            pred_t = d.get("delay", {}).get("prediction_s") or ct.get("prediction_time_s") or 0.0
            tr_fut = truth_at(t + pred_t)
            if tr_fut is None or bi >= len(tr_fut["armors"]):
                continue
            true_fut = tr_fut["armors"][bi]["position"]

            aim = ct.get("control_target_position")
            if not aim:
                continue
            # 完美指令(对预测点) —— aim 在 shooter/tracker 系(z 已减 0.405)
            sp = solve_pitch(aim[0], aim[1], aim[2])
            # 完美指令(对真实未来点) —— 真值在世界系,z 减 0.405 转到 shooter 系
            st = solve_pitch(true_fut[0], true_fut[1], true_fut[2] - SHOOTER[2])
            if sp is None or st is None:
                continue
            yaw_p = yaw_to(aim[0], aim[1])
            yaw_t = yaw_to(true_fut[0], true_fut[1])
            cmd_yaw = math.radians(cmd["yaw_deg"])
            cmd_pitch = math.radians(cmd["pitch_deg"])
            res["err_solver_yaw"].append(math.degrees(ang_diff(cmd_yaw, yaw_p)))
            res["err_solver_pitch"].append(math.degrees(cmd_pitch - sp[0]))
            res["err_pred_yaw"].append(math.degrees(ang_diff(yaw_p, yaw_t)))
            res["err_pred_pitch"].append(math.degrees(sp[0] - st[0]))
            res["err_total_yaw"].append(math.degrees(ang_diff(cmd_yaw, yaw_t)))
            res["err_total_pitch"].append(math.degrees(cmd_pitch - st[0]))
            # 云台滞后: 实际云台角 vs 指令
            res["gimbal_lag_yaw"].append(math.degrees(ang_diff(tr["gimbal_yaw"], cmd_yaw)))
            res["gimbal_lag_pitch"].append(math.degrees(tr["gimbal_pitch"] - cmd_pitch))

            if cmd.get("fire_advice"):
                n_fire += 1
                # 出膛时刻实际云台方向 -> 对真实未来板的角误差
                fire["err_fire_yaw"].append(math.degrees(ang_diff(tr["gimbal_yaw"], yaw_t)))
                fire["err_fire_pitch"].append(math.degrees(tr["gimbal_pitch"] - st[0]))
                # 弹丸对真值板平面的最近距离(粗略: 未来时刻板位, 弹道扫掠)
                v0 = [V * math.cos(tr["gimbal_pitch"]) * math.cos(tr["gimbal_yaw"]),
                      V * math.cos(tr["gimbal_pitch"]) * math.sin(tr["gimbal_yaw"]),
                      V * math.sin(tr["gimbal_pitch"])]
                tf = st[1]
                best = 1e9
                steps = 60
                pos = list(SHOOTER)
                vel = list(v0)
                for k in range(steps + 1):
                    tt = tf * k / steps
                    if k > 0:
                        # 子步积分(重力+二次阻力)，与仿真计分器一致
                        sp_ = math.sqrt(sum(c * c for c in vel))
                        acc = [-K_DRAG * sp_ * vel[0],
                               -K_DRAG * sp_ * vel[1],
                               -G - K_DRAG * sp_ * vel[2]]
                        dt_ = tf / steps
                        vel = [vel[j] + acc[j] * dt_ for j in range(3)]
                        pos = [pos[j] + vel[j] * dt_ for j in range(3)]
                    p = pos
                    for a in tr_fut["armors"]:
                        rel = [p[j] - a["position"][j] for j in range(3)]
                        dn = sum(rel[j] * a["normal"][j] for j in range(3))
                        # 平面内偏移
                        u = sum(rel[j] * a["width_axis"][j] for j in range(3))
                        v_ = sum(rel[j] * a["height_axis"][j] for j in range(3))
                        # 板半宽/半高(从角点估)
                        c = a["world_corners"]
                        hw = math.dist(c[0], c[1]) / 2
                        hh = math.dist(c[1], c[2]) / 2
                        du = max(0.0, abs(u) - hw)
                        dv = max(0.0, abs(v_) - hh)
                        dd = math.sqrt(dn * dn + du * du + dv * dv)
                        best = min(best, dd)
                fire["miss_dist"].append(best)

    def line(name, vals, unit=""):
        if not vals:
            print(f"{name:24s} n=0")
            return
        mean = sum(vals) / len(vals)
        rms = math.sqrt(sum(v * v for v in vals) / len(vals))
        print(f"{name:24s} n={len(vals):5d} mean={mean:+.4f} rms={rms:.4f} "
              f"p50={pct(vals,0.5):+.4f} p90={pct(vals,0.9):+.4f} {unit}")

    print(f"tracking frames={n_frames} fire_frames={n_fire}")
    line("center_err(m)", res["center_err"])
    line("center_err_xy(m)", res["center_err_xy"])
    line("vyaw_err(rad/s)", res["vyaw_err"])
    line("solver_yaw(deg)", res["err_solver_yaw"])
    line("solver_pitch(deg)", res["err_solver_pitch"])
    line("pred_yaw(deg)", res["err_pred_yaw"])
    line("pred_pitch(deg)", res["err_pred_pitch"])
    line("total_yaw(deg)", res["err_total_yaw"])
    line("total_pitch(deg)", res["err_total_pitch"])
    line("gimbal_lag_yaw(deg)", res["gimbal_lag_yaw"])
    line("gimbal_lag_pitch(deg)", res["gimbal_lag_pitch"])
    line("FIRE yaw(deg)", fire["err_fire_yaw"])
    line("FIRE pitch(deg)", fire["err_fire_pitch"])
    line("FIRE miss_dist(m)", fire["miss_dist"])
    if fire["miss_dist"]:
        hit = sum(1 for m in fire["miss_dist"] if m < 0.02)
        print(f"analytic would-hit(<2cm): {hit}/{len(fire['miss_dist'])}")


if __name__ == "__main__":
    if len(sys.argv) > 3:
        V = float(sys.argv[3])
    if len(sys.argv) > 4:
        K_DRAG = float(sys.argv[4])
    main(sys.argv[1], sys.argv[2])
