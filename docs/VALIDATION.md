# ROS-free 仿真验证记录

验证日期：2026-07-11

## 环境

- Webots R2025a
- ONNX Runtime CUDA Execution Provider
- 相机 1440x1080，桥接周期 32ms
- 目标默认自旋 6rad/s
- 控制策略 `predicted`，弹道模式 `local`

## stationary 修复后采样

20 秒算法采样，769 帧与 Webots truth 按 sequence 完整对齐：

| 指标 | 结果 |
| --- | ---: |
| PnP range / nearest armor truth 中位数 | 0.962 |
| 深度比 P10 / P90 | 0.920 / 1.001 |
| 重投影误差中位数 | 2.25 px |
| TRACKING 帧 | 198 |
| TEMP_LOST 帧 | 196 |
| TEMP_LOST 中 `mode=-1` | 0 |

`mode=-1` 的 375 帧均对应没有活动 tracker，TEMP_LOST 已连续输出有效云台参考且不开火。

## moving / orbit 入口

两个入口均实际启动 Webots、生成图像/真值并运行 ROS-free 检测跟踪链：

| 场景 | 对齐帧 | 有效控制帧 | 深度比中位数 |
| --- | ---: | ---: | ---: |
| moving | 667 | 45 | 0.951 |
| orbit | 657 | 142 | 0.968 |

moving 默认组合包含 6rad/s 自旋、横向运动和较高视觉变化，tracker 初始化与 commit gate
仍频繁重建。该问题不再阻塞 IO/坐标/控制闭环，但需要作为后续参数调优基线。

## 射手原点与执行模式修复

独立 ballistic oracle 发现控制器曾直接对世界坐标求 pitch，未减去实际射手高度约 0.405m：

- 修复前算法 pitch 约 +7.9°，物理理想值约 -4.0°，系统偏差约 11.9°。
- `predicted` 策略还错误配合了 Webots `mpc_state`，派生速度前馈可达约 21.8rad/s。

修复内容：

- 二进制桥控制帧改为射手原点、odom 对齐轴；相机平移改为相对射手的外参。
- Webots 默认 `WEBOTS_GIMBAL_COMMAND_MODE=position`；只有上游切换为 MPC 才使用
  `mpc_state`。
- Webots 启动时默认清理残留 `gimbal_command.bin`，避免序号重启被旧包屏蔽。
- truth 增加命令消费序号，oracle 按实际消费帧比较真实云台状态。

最终 stationary 6rad/s 隔离复测：匹配选板 yaw/pitch 中位误差 0.291°/-0.134°，弹道
脱靶中位数 0.059m、P90 0.097m，命中率 0.610。

## 复现

先启动任一 Webots 场景，再运行：

```bash
cd /root/hfut_auto_aim
./scripts/run_diagnostics.sh
./tools/diagnose_tracking.py --output /tmp/diagnostics_summary.json
```

## 整车防抖与预测修正（2026-07-17）

背景：gestalt 模式下两个突出问题——(1) 近静止靶的观测微抖带动整车状态，引发异常切板；
(2) 小幅快速运动被匀速外推放大，导致提前瞄准。本轮修复（commit 状态透出、motion guard
协方差同步与角向 guard、选板时间滞回、限幅 CA 外推、预测时域含实测处理延迟）后，
armor_pose 桥模式三场景冒烟：

### 静止靶（SPIN_RATE=0，45s，1313 有效控制帧）

| 指标 | 结果 |
| --- | ---: |
| 装甲板切换次数 | 0 |
| 线速度估计中位 / 最大 | 0.000 / 0.000 m/s（死区抑制） |
| 预测提前量 | 0.000 m |
| 实测处理延迟中位 | 0.3 ms |
| 云台 yaw 指令步进中位 / 最大 | 0.0004° / 0.0021° |

### 原地旋转靶（3 rad/s，45s，1302 有效控制帧）

| 指标 | 结果 |
| --- | ---: |
| 切板次数 / 节奏 | 79 次，间隔稳定 16–17 帧（理论 16.4），抖动切换 0 次 |
| yaw 角速度估计中位 | 3.002 rad/s（真值 3.0） |
| 线速度估计中位 / P95 | 0.040 / 0.110 m/s |

### 横向快速小幅抖动（±0.3m、2 m/s、4 m/s²，60s/轮）

预测落点精度（predicted_center 对 T 后真值）与命中率 A/B：

| 外推 | 误差中位 | 误差 P90 | 误差最大 | 命中率 |
| --- | ---: | ---: | ---: | ---: |
| 限幅 CA（新） | 0.0043 m | 0.0189 m | 0.0233 m | 0.423（355/859） |
| CV（旧） | 0.0151 m | 0.0200 m | 0.0590 m | 0.429（401/957） |

CA 外推在换向段的落点误差显著更小（最大 2.3cm 对 5.9cm），近距离命中率持平；
误差的角量化在远距下收益更大。单测新增 `tracker_motion_guard`（guard 缩放/重建、
角向 guard、协方差同步）与 `armor_selector_hysteresis`（滞回保持与真实切换放行），
`ctest` 5/5 通过。

## 传统灯条检测器移植与回退路径（2026-07-18）

`armor_detector`（灯条检测 + LeNet 数字分类 + PCA 角点精化）已移植为
`armor_detector_traditional`（方案 B：检测产出 ArmorDetection，位姿估计复用共享
ArmorPoseEstimatorAdapter，不引入 g2o/Sophus）。`detector.impl` 支持
`nn | traditional | auto`（NN 初始化失败自动回退）。

3 rad/s 原地旋转靶、vision 模式、45s 对照：

| 检测链 | TRACKING 占比 | TEMP_LOST | 检出/帧 | 切板次数 |
| --- | ---: | ---: | ---: | ---: |
| traditional（binary_thres=90） | 73%（839/1157） | 282 | 1.06 | 65 |
| traditional（binary_thres=100） | 45%（572/1258） | 659 | 1.07 | 205 |
| nn | 41%（470/1153） | 672 | 1.35 | 218 |

LeNet 数字分类在仿真图上全部判为 "4"、置信度 ~1.0，正确且稳定。注意仿真灯条
亮度远低于真车过曝 LED（灰度峰值 ~143），仿真需 `binary_thres: 90`，真车应恢复
160。`auto` 回退实测：NN 模型路径失效时自动切到 traditional，15s 379 帧正常跟踪。

## 配置文件主题拆分（2026-07-18）

`configs/auto_aim.yaml` 与 `configs/gimbal_pipeline.yaml` 按影响面拆分为 5 个文件
（同目录）：`gimbal_pipeline.yaml`（总控开关：global 模式、跟踪器实现与后端、
selector、控制策略）、`simulation.yaml`（桥接/外参/弹速）、`detector.yaml`
（NN/传统检测）、`tracker.yaml`（跟踪/滤波/输出平滑 + tracking 防抖覆盖）、
`controller.yaml`（解算/延时/开火/MPC）。参数层按 tracker→controller→master
顺序深合并（master 优先）。重复项已合并为单一来源：`prediction_extra_s`、
`output_stabilizer`（→ smoother:）、`blocked_robot_ids`、`bridge.path`/
`input_mode`/`detector.impl`（→ global:）。拆分校验：armor_pose 30s
（TRACKING 638/650、切板 39 次合节奏、yaw_rate 2.994≈3.0）、默认 vision
模式 NN 链 15s 正常。

## 预测偏差真值测量与 OneEuro 速度自适应调参（2026-07-18）

方法：bridge 真值（target_truth.jsonl，统一至 shooter 原点）与诊断
（control_target.predicted_center / control_target_position）按 sim_time 对齐，
比较当前估计、t+T 预测与最终瞄准点偏差。场景：armor_pose 桥模式。

发现：原 output_stabilizer 参数（min_cutoff≈1Hz、beta≈0.05）使 OneEuro 近似
静态平滑，1m/s 横移时相位滞后 ~0.11m；将 beta 提高到 3.0/2.0（速度自适应）
后静止时 cutoff 仍在 ~1Hz（防抖不变），运动时 cutoff 随速抬升（弱平滑）。

| 场景（3 rad/s 自旋） | 指标 | 调参前 | 调参后 |
| --- | --- | ---: | ---: |
| 静止 | current_center err | 0.033 m | 0.031 m |
| 静止 | predicted_center err | 0.035 m | 0.032 m |
| 静止 | 瞄准点 err | 0.051 m | **0.015 m** |
| 静止 | 切板 / 抖动切板 | 正常节奏 / 0 | 63/理想64 / 0 |
| 移动(±1.5m,1m/s) | current_center err | 0.111 m | **0.048 m** |
| 移动 | predicted_center err | 0.126 m | **0.067 m** |
| 移动 | 瞄准点 err | 0.114 m | **0.057 m** |

smoother 全关对照（移动场景）：current 0.049 / predicted 0.068 / aim 0.061，
与调参后开启状态相当——速度自适应保留了静止防抖收益且几乎不引入滞后。

## vision 链路稳定性根因修复（2026-07-18）

现象：vision 模式下跟踪输出闪烁（TRACKING 仅 35%，状态每 ~1.6 帧翻转一次，
1050 帧中 664 帧全部假设被 NIS 门拒止），而原始 NN 检出本身稳定（置信度 ~0.95，
零掉检）。按时刻曲线分析（非抽样平均）定位两层根因：

1. **量测噪声失配**：`observation_noise_scale=0.35` 是按 armor_pose 注入噪声
   （0.02m/0.05rad）标定的，vision PnP 实测噪声 ~0.09m/12°（2m 距离），
   有效 R 小了 ~5x，NIS 膨胀 ~25x 全部超门。修复：`tracking.observation_noise_scale`
   改为按模式配置 `{vision: 1.5, armor_pose: 0.35}`。
2. **系统性深度偏差 −6.9cm**：NN 关键点在暗色 mesh 上比真实板角外扩 ~3.7%，
   单 yaw 精化按模型板宽求解使深度系统性偏小 3.7%。实测关键点缩放 A/B：
   1.03 → −10.6cm，0.9635 → **−0.4cm**。修复：`simulation.yaml` 新增
   `bridge.webots.keypoint_scale: 0.9635`（仅 PnP 用，webots 渲染专用；gestalt
   仍用自己的 1.192，真车需按标定另设）。

综合结果（vision, 3 rad/s 旋转靶）：

| 指标 | 修复前 | 修复后 |
| --- | ---: | ---: |
| TRACKING 占比 | 35%（364/1050） | **95%（730/765）** |
| track_state 翻转 | 655 | 49 |
| NIS 全拒止帧 | 664 | 32 |
| current_center err | 0.141 m | **0.054 m**（噪声下限 ~0.057） |

调试入口：`HFUT_DEBUG_OBS=1` 时 bringup 打印每块观测板的 world pos/yaw
（`pipeline.cpp` updateTracking），可与真值板逐帧对齐。

## gestalt 实测问题修正（2026-07-18）

针对 gestalt 实测的三类问题（参数调整，全部经 Webots 冒烟复测无回归）：

1. **整车轻微滞后（~0-0.5s 体感）**：主要来源为输出平滑/滤波相位滞后（速度
   自适应后已大幅消除）与传输+采集延迟。处理：`controller.delay.prediction_extra_s`
   0.0→0.03 补偿残余滞后（延迟链已含单帧实测算法耗时）；实机上可按 0.01-0.02
   步长继续上调，旋转靶提前量明显过大则回调。冒烟：aim err 均值 0.040m
   （extra 带来的提前约 1.8cm/3rad/s）。
2. **高速目标脱离视野**：guard 限幅压住高速估计是主因。`max_linear_speed_mps`
   3.0→5.0（3m/s 限幅会直接削掉预测提前量）；`temp_lost_velocity_half_life_s`
   0.12→0.30（突发提速恰逢 NIS 拒止时不再把跟随所需速度迅速衰减）；预测外推
   加速度限幅 4.0→6.0 m/s²。另注意 gestalt 相机 FOV 仅 25°（代理
   `rgbCamera.applySettings.fovDegrees` 可调，视野换分辨率）。
3. **第一视角归属**：代理链路两级均支持 `--entity-id/--team-id`
   （`run_gestalt_stdio.sh` → `gestalt_bridge_windows.py`），当前默认
   `66000005/0`（红方 HACHISEN，class 1004）。切蓝方哨兵：`--team-id 1`
   （蓝方 entity id 需按游戏 entity 配置确认，手册只验证了红方 66000005），
   并把 `simulation.yaml` 的 `bridge.gestalt.enemy_color` 改为 "red"
   （传统检测器颜色默认跟随 gestalt，无需另改）。

## 随机移动靶系统调节与难度阶梯（2026-07-18）

协议：seed=42 固定轨迹、每次配置重启仿真（同相位）、算法 60s、armor_pose 桥模式。
DPS 统一按 hits/60s·20 折算（score.txt 的 dps 字段受空闲期稀释）。随机轨迹随
相位变化——同仿真内不换配置连测会把相位差误当配置差异（协议污染，本次
sigma=5.0 的早期"灾难"结论即由此产生，未复测前不作数）。

### 调节结论（当前生效配置）

| 项 | 值 | 说明 |
| --- | --- | --- |
| singer_sigma | 2.5 → **3.5** | 匹配高机动量级；5.0 早期失败结论被协议污染，待干净复测 |
| shooting_range_width/height | 0.1（试过 0.15 回退） | 0.15 开火占空 +10pt 但命中率 −6pt，DPS 净降（279→263） |
| prediction_extra_s | 0.03 → **0.05** | R3 难度下 DPS 249→261、命中 90.5→92.6% |
| 命中概率模块 | **保持关闭** | 默认参数整局零授权（见瓶颈 B2） |

### 难度阶梯（最佳配置下）

| 档 | 目标参数（spin/v/a/sa） | 有效DPS | 命中率 | 开火占空 | TRACKING | NIS拒止 | aim p99 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 基线 | 3 / 1.5 / 3 / 4 | 311 | 91.5% | 92.1% | 94% | 4 | 0.12 m |
| R1 | 4 / 2.0 / 4.5 / 6 | 314 | 91.5% | 93.1% | 98% | 17 | 0.21 m |
| R2 | 5 / 2.5 / 6 / 8 | 279 | 93.3% | 86.7% | 90% | 149 | 0.24 m |
| R3 | 6 / 3.0 / 8 / 10 | 261 | 92.6% | 84.5% | 96% | 4 | 0.21 m |
| R4 | 8 / 3.5 / 10 / 12 | **177** | 81.3% | 69.9% | 86% | 191 | **0.42 m** |

R3 以内整体稳健，DPS 下降主要由开火占空驱动；**R4 为当前瓶颈区**。

### 瓶颈记录（按约定只分析不修复，待研判）

- **B2 命中概率模块不可用**：`fire.probability.enable: true`（默认窗口/门控）下
  整局 `fire_advice=0`（1294 帧、TRACKING 96% 也无一授权）。不是保守，是
  完全不放行；疑似其 p_hit 计算在本模式下恒不通过（协方差可用性/门限）。
  建议专门调试后再评估是否启用。
- **B3a guard 加速度限幅低于真实机动**：R4 目标 accel=10 m/s²，而
  `motion_guard.max_linear_acceleration_mps2: 8.0` 直接削掉真实加速度，
  CA 外推（限幅 6.0）在偏低估计上提前量不足。候选动作：guard 提到 12、
  外推限幅同步上调——待研判。
- **B3b 过程模型量级封顶**：Singer σ=3.5 ≈ 3.5 m/s² 机动量级，R4 的 10 m/s²
  使 NIS 拒止回升到 191、轨道移除 7 次。sigma 5.0 是否可行需干净复测
  （被污染的失败结论不能引用）。
- **B3c 高机动速度估计过冲**：R4 下 |v| 估计 max 4.49 m/s（目标真实 3.5，
  guard 5.0 未触发）。速度噪声尖峰直接恶化预测提前方向。
- **B4 开火占空是 DPS 主驱动**：R1→R4 DPS 314→177 的下降中，占空
  93%→70% 的贡献大于命中率变化。若要 DPS 优先，提升机动段瞄准稳定性
  （而非放宽窗口）是正道。

## 随机靶优化第二轮（2026-07-18，B3 系列处置）

按研判意见执行：guard 加速度限幅 8→12 m/s²、预测外推限幅 6→8 m/s²、
singer_sigma 干净复测（同 seed 同相位、重启仿真）。

| 难度 | 配置 | 有效DPS | 命中率 | NIS拒止 | 移除 | aim mean/p99 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| R4(8/3.5/10/12) | sigma 3.5 + guard 8 | 177 | 81.3% | 191 | 7 | 0.097/0.42 m |
| R4 | sigma 4.0 + guard 12 | 214 | 87.1% | 144 | — | 0.083/0.38 m |
| R4 | **sigma 5.0 + guard 12** | **235** | 85.0% | **52** | **3** | 0.084/0.37 m |
| R3(6/3.0/8/10) | sigma 3.5 + guard 8 | 261 | 92.6% | 4 | 2 | 0.062/0.21 m |
| R3 | sigma 5.0 + guard 12 | 276 | 91.3% | — | — | 0.087/0.42 m |
| 良性(3rad/s 静止) | sigma 5.0 + guard 12 | — | — | — | — | current 0.036 / aim 0.035 m，切板 53/理想53 |

结论：**singer_sigma 定为 5.0**（R4 DPS +33%，R3 持平略升，良性场景代价
aim 1.5→3.5cm 且切板节奏无损），guard 12 保留。TEMP_LOST 自适应惯性外推
（运动目标 coast 0.5s / 静止 0.15s）已上线，selector 距离门 6.0m 生效，
outpost 屏蔽恢复（此前被误置为 `[""]`）。

## 整车 roll/pitch 姿态估计上线（2026-07-18）

实现：`max_entropy_tracker/utils/body_attitude_estimator.hpp`——由装甲板 PnP
全姿态恢复底盘竖直轴（plate up 按挂载角 `tracking.attitude.mount_pitch_deg`
去倾，EMA 平滑 `ema_alpha`），结合跟踪 yaw 解算整车 pitch/roll。输出：
`TrackedRobot.center_pose`（position+attitude 四元数，`full_state_valid` 保持
false 不改变平面投影）、调试 overlay "Body R/P"、诊断 `body_attitude`。
直接位姿（armor_pose）帧的 yaw-only 四元数不喂养估计器（pose_estimate_mode==0
保护）。

平地验证（vision, 3 rad/s 旋转靶）：pitch 均值 +4.5° sd 7.4°、roll 均值 −2.4°
sd 10.4°。性质：单板 PnP 姿态噪声 ±8-14° 与板相位偏置决定的慢信号——可读
坡度（>10°）与刹车晃动，不是精密姿态。配置：`tracking.attitude.mount_pitch_deg`
（名义 15°；webots mesh 经 PnP 表观 ~7°，当前取 7）、`ema_alpha` 0.02
（约 1.3s 窗口）。后续可加权：按板面积/BA 协方差、按板类标定挂载角。

## vision 三问题根因与修复（2026-07-18）

**问题2（静止小幅抖动→假旋转）已解决**。根因不是观测噪声本身，而是旋转通道
过程噪声常量化：`spin_process_noise_delta_rate` 任何单值都无法两端兼顾——
1.0 静止干净但 3 rad/s 永不收敛（中位 0.245），2.0/3.5 旋转正常但静止假旋转
（中位 0.79-1.48 rad/s）。修复（两级）：
1. yaw 观测噪声朝向自适应（sp_vision_25 target.cpp:191 同款）：侧对板 yaw
   降权 log(facing+1)+1（1x→1.94x），作用于 UKF 单/双板更新的 R 与门控；
2. **旋转过程噪声按"实测转速"自适应**：单板提交后 EMA 维护 center-yaw 差分
   转速（0.10 增益、±15 rad/s 截断、前 25 帧全带宽），predict 时按
   `clamp(0.3+0.35·|ω_meas|,0.3,1)` 缩放旋转 Q 块。静止时 yaw_rate 中位
   0.000（>0.5 仅 6.9%，原 64%）；3 rad/s 旋转后半段收敛 3.6（切板 30/理想28，
   无抖动切换）。delta_rate 保持 3.5 作全带宽上限。

**问题1（高速逼近 pitch 响应慢/跟丢）部分缓解**。取证：TRACKING 段俯仰滞后
+1.5~+3.6°（±7° 垂直 FOV 吃掉近半裕度），<1.2m 时检测 95% 灭失——先在
~1.5m 俯仰误差越过垂直视场 → 装甲出画 → 检测黑屏 → 云台无目标可俯 →
永久跟丢的死锁。已做：OneEuro 速度自适应提速（vel_beta 3、d_cutoff 3），
2-3m 俯仰误差从 +3.4~+4.8° 降到 +2.4~+4.2°。残余硬约束：25° 视场几何
（垂直 ±7°）+ 近距离大板检测失效，属场景物理边界；候选（未做）：巡扫器按
最后已知方向带俯仰搜索、近距离大板姿态特殊处理。

**问题3（观测滞后）**：同问题1的 OneEuro 提速覆盖，速度向量误差在 |v|>1.5
时仍约 1.1 m/s（机动本质），逼迫段俯仰滞后见上。

借鉴记录（sp_vision_25）：R 朝向自适应（已采用）；yaw 穷举重投影搜索
（solver.cpp:196-218，候选，正对静止代价平坦需先验补救）；飞行时间迭代收敛
（aimer.cpp:78-116，我方 flight_time_iters=2 可参照改收敛式）；NIS 滑窗
发散重置（tracker.cpp:83-90，候选）。

## 纵深变化短暂丢失：根因与拒止重置（2026-07-18）

纵深振荡取证（x 轴 0.5-3.5m 往返 2m/s + 3 rad/s 自旋，vision）：丢失由三层
叠加——(a) <1m 检测黑屏（99% 无检出，FOV 物理边界，每个接近周期必经）；
(b) 换向/远离段状态过冲 + 小板检测下降引发拒止簇；(c) **黑屏后重捕获过慢**：
检出恢复后到 TRACKING 中位 5 帧、p90 25 帧——失真状态靠协方差膨胀缓慢自愈。

修复（sp_vision_25 tracker.cpp:83-90 同款拒止重置）：连续门控拒止 ≥
`tracker.reject_reset_streak_frames`(12) 且当帧 top1 假设仍过 NIS 门时，用
当前观测+已学结构重新初始化，把可观测条件下的重捕获从 ~25 帧压到几帧。
复测：重捕获延迟中位 5→**3 帧**；长 p90 尾部仍存在，由 (a) 的检测灭失主导
（非跟踪器问题）。R3 常规难度回归：TRACKING 91.9%、重置零误触发，无副作用。

纵深/近距离的硬性边界（不改算法无法消除的部分）：25° 视场垂直 ±7°、
<1m 大板检测失效、自旋侧对相位检测闪烁。要消除需更广视场或近距离专门
处理链（记录待研判）。

## 难度矩阵与全局调优（2026-07-19，全部 vision 模式）

### 矩阵设计

三场景 × 四档难度（`hfut_auto_aim_sim/config/difficulty_levels.env`）：
spin 2/4/6/8 rad/s、横移 0.5/1.0/1.5/2.0 m/s、加速度 1.5/3/5/8 m/s²。
运行器 `hfut_auto_aim_sim/run_difficulty_matrix.sh`（每格重启仿真、
真值+相机双流就绪检查、帧数不足自动重试、保存诊断/真值/score），
报告 `hfut_auto_aim_sim/tools/matrix_report.py`（TRACKING%、det0%、
当前/预测中心误差、转速估计、命中率、有效 DPS=hits×20/算法秒数）。

### 根因修复（按发现顺序）

1. **桥接指令序号死锁**（hfut_auto_aim_sim）：bridge 只收 seq 单调增的
   指令，pipeline 重启后 seq 归零 → 指令全被静默丢弃、云台冻结在原点。
   修复：seq 回退即识别为新写入会话并重新同步。
2. **拒止重置保留污染结构**：掉帧 → 板混淆 → 结构 r1/r2 被双板更新带偏
   （0.15/0.20→0.12/0.236）→ 重建拒止触发重置，但原样保留污染结构，
   误判不自愈。修复：重建拒止占多数时改用标称结构重新播种；普通 NIS
   拒止仍保留已学结构。
3. **EMA 换板伪迹**：换板滞后使 center-yaw 单帧回跳 -15 rad/s，污染实测
   转速 EMA（spin 2 时 EMA 游走 ~0.5）。修复：|inst|>12 rad/s 判伪迹跳过。
4. **对称板陷阱（核心）**：静止 yaw 模型 + 四块全同装甲板 = 观测总能匹配
   某块板 → 提交持续成功、拒止串永不触发、ω 永久锁 0（L4 多发：60s 真值
   8.0 读出 0.32，带宽/见证 EMA 均无法自救，因为换板行走吸收了旋转）。
   修复：**无模型方位见证**——观测板相对估计中心的方位角逐帧差分
   （±90° 换板帧 |delta|>0.7 rad 跳过），与滤波 ω 持续背离 >2 rad/s 时
   对 yaw_rate 状态做有界推移。静止档 EMA≈0±0.3 无误触发。
5. **带宽驱动切换**：自适应旋转 Q 的驱动从 center-yaw EMA（静止时振荡
   ±2 rad/s、顶住假旋转）切到方位见证 EMA。静止档 ω 尾巴
   p90 3.79→0.51 rad/s、max 11.5→4.3。
6. **空转保持**（controller.idle_hold_s=2.0）：目标丢失后保持最后瞄准
   方向 2s 再回零，扫掠目标更快重入视场。

### 矩阵结论（v8 全局值）

| 场景 | L1 | L2 | L3 | L4 |
|---|---|---|---|---|
| 定点旋转 eDPS | 229 | 261 | 164 | 67 |
| 平移旋转 eDPS | 139 | 99 | 58 | 45 |
| 随机走位 eDPS | 213 | 118 | 102 | 55 |

转速估计全档准确（2/4/6/8 → 1.97/4.50/6.63/9.03，高档略过冲 ~10%）；
TRACKING 除 random_L4（54%）外 ≥77%。总 eDPS 1455(v2 基线)→1550。

### 已知残余边界（记录待研判，勿立刻修）

- random_L4（2 m/s+8m/s²+7rad/s）长黑屏：近距 <1.2m 检测盲区与高速
  换向逃逸各占一半；近距离检测与搜视策略是场景级问题。
- 转速高档过冲 ~10%（方位见证 EMA 系统性偏快），spin 6/8 命中率
  受此影响约 5-10 个百分点。
- 静止档 ω 尾巴残留 p90 0.5 rad/s。
- prediction_extra_s 全局不敏感（0.02/0.05 噪声级差异，0.12 明确有害），
  取 0.05；移动靶 14-20cm 预测残差由速度估计滞后主导，非时域参数可解。

## 远距清除、选板与 MPC 核查（2026-07-19 第二批）

### 问题1：高速远离时观测器不断清除目标

取证链（spawn 5.5m、4-7m 往复 2m/s + spin 3）：
- 6m 外：selector.max_distance=6 距离门按设计丢弃（用户要求保留）。
- 4-6m 内：TRACKING 仅 9-23%、翻转 142 次/60s。根因是**检测 ID 误读**——
  NN 数字分类在远距把同一台车判成 3/4/5/outpost（obs 日志实锤），每个
  误读 ID 凭空建新轨迹、真轨迹被饿死；叠加 NIS/后验门限按 2m 标定
  （sigma 固定），远距观测直接全部拒止。

修复（全部验证）：
- `tracking.id_association_max_distance_m: 0.6`：观测按空间位置归并到
  已有轨迹（ID 无关），误读 ID 不再产生幻影轨迹。
- 观测噪声随距离缩放：位置 sigma 横向 ×(r/2)、纵深 ×(r/2)²（≤3x），
  posterior_sanity 的 center_jump 同倍放宽。
- R_yaw 随距离二次放大（3m 起 ≤25x）：远距 PnP yaw 实测误差 ±100-150°，
  不再参与门控与更新，位置信息独立工作。
- 近 π yaw 折叠（>3m 且 |innov_yaw|>2.6 rad）：单板 PnP 180° 翻转折叠。
- 重捕获宽容 `ukf_v1.gate.init_relax: 3.0`：非 TRACKING 期间门限放宽。
- `detector conf_threshold 0.5→0.35`。
结果：4-5m TRACKING 23%→48%、5-6m 9%→24%；拒止风暴消失；标准矩阵
（v9b）总 eDPS 905→950，无回归。

### 问题2：一块板略正对、一块大角度时误选难命中板

取证（spin 0.5 慢旋）：选中板偏离真值朝己板 >45° 的帧占 39.5%，且不是
抽搐而是长时间停在错误板上。根因：facing 门 65/85° 过宽（边缘板也入选）。
但全局收紧 40/60 又会：高速自旋下开火量被掐（DPS -20~40%）+角位两板
同时被过滤导致选板掉到 -1 抽搐（5% 帧）。

最终方案：**转速自适应朝向门**——|v_yaw| 0.5~3.0 rad/s 线性插值，
低速 40/60°、高速 65/85°（`solver.facing_enter/exit_low_spin_angle`）。
结果：低速错板率 39.5%→18.0%、切换 16 次（原 8）；高速档 eDPS 回到
基线（stat_L2 260、L3 179、L4 63）。

### MPC / 轨迹优化器迁移核查

迁移状态：**已完整迁移**——mpc_control_strategy.cpp 与
mpc_reference_generator.cpp 与 ROS 仓逐字节一致，qpOASES 已链接、
策略已注册（gimbal_strategies_["mpc"]），总控 strategy: "mpc" 可选。

冒烟实测（moving_spin L3）：可运行但性能不达标。修复两个配置问题：
- `mpc.vel_clamp.max_linear_speed 1.0→3.0`（原值把 >1m/s 目标的参考
  轨迹限死，必然跟丢）；
- `mpc.N 50→16`（N·dt 2.5s 时域内 CA 参考在振荡目标换向后发散，拖走云台）。
修后 TRACKING 59%→77%，仍低于 predicted 的 96%；指令 yaw 误差中位 5°
（远超开火窗口），开火基本不触发。

结论：MPC 代码可用、需专项调参（参考轨迹时域/延时标定/开火协调）才能上；
当前保留 predicted。gestalt 弱云台场景的理论优势（QP 内加速度/速度约束
可行轨迹）成立，但需先把瞄准中值误差从 5° 压到 1-2° 再评估。

## 装甲板三维布局：可视化与预测的安装面倾斜修正（2026-07-19）

### 背景（布局约定）
四板 0/2、1/3 对板分组（同组共安装平面、两组平面平行可不重合 →
r1/r2/dza）；安装平面可不平行地面；旋转轴可不垂直安装平面；板面与布局
面法向成 15° 安装角（webots 靶车 truth normal z=+0.259 实测）。

### 取证结论
1. **15° 板面安装角**：debug overlay 此前已正确——`armors_offset` 姿态
   含 15°（`generateArmorsOffsetFromProfile`），overlay 板 height 轴
   z 分量实测 0.966=cos15°，width 轴水平（HFUT_DEBUG_AXES 打印验证）。
   ~2m 处 15° 后倾仅 ~1° 视角差，视觉上不明显但几何正确。RViz marker
   路径（visualization.hpp）同为 yaw-only 中心系，未改（非调试主链路）。
2. **安装面倾斜被丢弃（问题2 实锤）**：`calculateArmorWorldPosesEigen`
   的中心系四元数默认 yaw_plane 策略 = Rz(yaw) 纯偏航，车体 roll/pitch
   在可视化与预测中都被抹掉。
3. **姿态估计器参数错误（关键）**：`tracking.attitude.mount_pitch_deg`
   原值 +7°，用实测 PnP 数据（840 帧）试算三种取值，chassis_up 与竖直
   偏差：+15°→28.5°、+7°→20.5°、**−15°→3.6°**。PnP 测量系下正确值为
   **−15°**（该系法线方向与 offset 档案系相反，注意与几何单测中
   offset 系的 +15° 区分）。平地姿态读数修正前 roll/pitch≈10-13°
   （应为 0），修正后 roll +0.3°、pitch +0.2°（p90<2.1°）。
4. **预测路径本就保倾斜**：`predict()` 用完整角速度向量积分初始姿态
   （非 Z 轴硬编码），layout_attitude_valid 透传；几何单测
   `armor_geometry_3d` 已覆盖（倾斜轴/零时/外推三种情形）。

### 改动
- `configs/tracker.yaml`：`attitude.mount_pitch_deg: 7.0 → -15.0`；
  `attitude.apply_to_geometry: false → true`（注释说明符号约定与取值
  证据）。启用后：姿态并入 center_pose + layout_attitude_valid=true →
  AUTO 投影自动走 full SE(3) → 可视化与预测都跟随真实安装面倾斜；
  无可信姿态时回退原 yaw_plane 行为。
- `apps/bringup_sim.cpp`：HFUT_DEBUG_AXES 调试打印（一次性的轴值核查，
  env 门控）。

### 验证
- 几何单测 `armor_geometry_3d` + 全部单测 7/7 通过。
- 矩阵回归（stat/moving L1-L3 六格）：stat_L1 263、stat_L3 242（均历史
  最佳），moving 各格在噪声带内，无回归。
- 倾斜靶车端到端仿真待做：当前 webots 靶车不支持整车 roll/pitch 参数
  （仅装甲板 15° mount pitch），如需端到端倾斜验证需在仿真仓加倾斜
  靶车型（记录为后续项）。

## 可视化/屏蔽/响应延迟/姿态符号批次（2026-07-19 第三批）

### 逐项结论
1. **估计可视化全程可见**：原调试 overlay 只在 control.valid（AIM 态）绘制
   整车估计板，非 AIM（跟踪/TEMP_LOST）时什么都不画，看起来像"估计器
   没在工作"。改动：pipeline 在选择结果处填充 `tracked_armor_poses`
   （含 width/height 轴与板尺寸），bringup_sim 在非 AIM 且轨迹存活时
   持续绘制估计板。
2. **屏蔽列表下沉 detector 级**：原 blocked_robot_ids 只在 selector 生效，
   被屏蔽目标仍建跟踪。现于 `updateTracking` 入口直接丢弃被屏蔽装甲板，
   跟踪/选择/姿态链全部不再接触（selector 级检查保留兜底）。
3. **PREDICT_ONLY（TEMP_LOST）响应慢**：原重置需 top1 过严格 NIS 门，
   大机动时所有假设都不过门 → 拒止到 lost_thres(20) 掉轨走慢重捕获。
   改动：top1 门限改用 init_relax 放宽门；另加 force 重置
   （streak ≥ lost_thres-4 时无门重置，结构改标称值重播）。实测 moving
   L3：TEMP_LOST 段中位 1 帧、最长 11 帧（原可到数十帧）。
4. **姿态估计符号自校准**：mount_pitch 符号依赖坐标系约定（offset 档案
   系 +15°、webots PnP 系需 −15°），按平台硬编码必错。改为按帧自校准：
   ±|mount_pitch| 两个候选取更竖直者（底盘倾角 <15° 时恒正确），
   mount_pitch_deg 恢复为物理值 +15.0、平台无关。平地读数
   roll +0.01°/pitch −0.27°（p90 ~1.3°）。
5. **单板结构学习试验（已回退）**：尝试 single_update structural_gain_r
   0→0.05 让结构在单板主导期慢速学习，矩阵实测轻微变差（r1/r2 随噪声
   游走：stat_L3 242→185、moving_L1 131→103），回退为 0（冻结）。
   单板目标的结构错误错位机制保留为未决项（见下）。

### 回归（v14，六格）
moving_L1 168（历史最佳）、stat_L1 260、stat_L2 240、moving_L2 66；
stat_L3 117（该格历次方差极大 117-242，取低值段）；moving_L3 48。
批次整体净中性偏正。

### 未决（gestalt 专属，需 gestalt 数据继续）
- **单板错位（估计错位.png）**：候选机制——(a) 单板主导期结构冻结在
  初始 0.15/0.20，机型不符（英雄）时四板模型无法拟合观测→持续重建
  拒止→模型错位；(b) gestalt PnP/外参与 webots 不同需单独标定。
  结构在线学习已证有害被回退；建议方向：按装甲尺寸类别（大/小板）
  给初始结构剖面，或 gestalt 上实测标定。
- **双方运动偏移（误差.png）**：帧变换本身正确（gestalt 元数据取采集
  时刻相机世界位姿）。残余=估计滞后（已知速度估计滞后瓶颈）；另需
  核查 gestalt 元数据中相机位姿与图像的时间戳是否对齐（若位姿滞后，
  应按 gimbal_yaw_vel×延迟补偿）。

## armor_pose 状态估计/跟踪/预测专项（2026-07-20）

本轮诊断日志位于 `/tmp/hfut_runs/` 对应场景目录，统一通过
`tools/diagnose_tracking.py` 汇总。关键修复与可复现实测：

- V1 双板结构增益 `1.0→0.25`，默认 `r1/r2=0.20/0.20`；避免高角度板使半径
  单帧跳变 5-8cm、连带丢弃有效运动更新。
- `dza` 按 panel 相位允许有符号 `[-0.15,0.15]`；`[r1,r2,dza]` 与
  `[r2,r1,-dza]` 是等价物理布局。纵深往返 + 8rad/s：1320 帧中 TRACKING
  1317、TEMP_LOST 0，结构等价误差 median 0.76cm / p90 1.25cm。
- 静止 yaw-rate 发布死区 `0.05→0.5rad/s`，内部状态不改。固定 yaw 45s：
  1297/1297 更新 commit、发布 yaw-rate 0 帧占比 100%，eDPS≈319，与前版约
  320 持平。
- `armor_angle_ema` 在板号跳变时清空，并要求 5 个连续有效同板样本才可触发
  对称陷阱修正。随机 3rad/s 启停 60s：静止真值 1248 帧中仅 4 帧残余，均在
  停转边界约 96ms 内；停转数秒后的旧 witness 假旋转消失。
- `posterior_sanity.max_center_jump=0.30m` 只用于 V1 近距重捕获，NIS/yaw/
  结构/重建门不变。固定仿真 8.032s 启动的 8rad/s 纵深 45s：1321/1321
  更新 commit、direct 空帧 0，22.0-22.5s 近距换向连续提交。

诊断工具现在还报告输入空窗、状态连续段、commit/reject 分类、真值静止时
yaw-rate 尾部和 90deg 等价结构误差。超过云台能力的 random L4（8rad/s）仍有
28% direct 空帧，归类为搜视/MPC 能力边界，不作为连续 armor_pose 滤波重置证据。

## armor_pose 发布器的 PnP 风格系统失真（2026-07-20）

`controllers/ros_free_camera_bridge/ros_free_camera_bridge.cpp` 在保留 v3
协议的前提下加入：

- `yaw_bias = yaw_gain * sin(view_angle)^exponent * camera_pull`，其中
  `camera_pull` 限制在 ±90deg，避免板号/法向符号异常产生非物理大跳；
- `p_biased = camera + ray * range * (1 + range_ratio * sin(view_angle)^exponent)`；
- 板面四元数同步绕世界 Z 轴施加同一 yaw bias；独立高斯项仍单独记录在
  `position_noise_std_m/yaw_noise_std_rad` 元数据中。

可调环境变量：`WEBOTS_DIRECT_ARMOR_YAW_BIAS_GAIN`（默认 0.25）、
`WEBOTS_DIRECT_ARMOR_RANGE_BIAS_RATIO`（默认 0.10）、
`WEBOTS_DIRECT_ARMOR_BIAS_ANGLE_EXPONENT`（默认 1.5）。

验证：桥接器构建成功；随机噪声置零、range ratio=1.0 的极端探针在
`view_angle=0.096rad` 测得距离比 1.0299，与 `1+sin(0.096)^1.5` 一致。
高角度的默认参数曲线仍需固定板 ID 的夹具回归，避免近邻物理板误匹配污染统计。

### 2026-07-20 现场补充：任务 2-6 回归

- **高视角 PnP 失真**：固定相机 yaw=0、目标 yaw=1.0、随机噪声全关，直接
  位姿记录按真值位置最近邻匹配。`view_angle=1.073rad` 板的测距比为 1.082、
  径向 yaw 偏差 -0.225rad；`view_angle=0.626rad` 板的测距比为 1.045、
  径向 yaw 偏差 +0.070rad。方向与视角单调性符合发布器的 PnP 系统误差模型。
- **屏蔽端到端**：`WEBOTS_DIRECT_ARMOR_NUMBER=outpost`，armor_pose 运行
  221 帧；直接输入全部为 outpost，`tracked_count` 最大 0，正常命令 0，
  开火 0，命令 mode 始终 `no_valid_measurement`。此前使用默认 bridge 目录
  读取了旧序号/旧记录的探针已废弃，复测使用同一显式目录通过。
- **非水平目标**：新增 `WEBOTS_ARMOR_TARGET_ROLL/PITCH`（整车斜面）与
  `WEBOTS_ARMOR_LAYOUT_ROTATION`（安装面相对底盘）两个独立嵌套 Transform。
  两类固定姿态探针的直接四元数法向与真值最近板误差分别为 `6.3e-5`、
  `2.3e-4`；各自 276 帧短闭环均无 `LOST`，TRACKING 262 帧，姿态输出约为
  `(-10deg,+5deg)` 与 `(-10deg,0deg)`（估计器坐标符号）。
- **MPC 联调**：临时配置 `controller.strategy=mpc`，Webots
  `WEBOTS_GIMBAL_COMMAND_MODE=mpc_state`，横移+2rad/s 运行 514 帧。桥接器新增
  无效命令清空 active 状态、50ms horizon/lead 对齐、位置误差反向速度丢弃和
  MPC 专用前馈限幅（默认 yaw/pitch=2/3rad/s）；TRACKING/TEMP_LOST 提升到
  333/41，direct 空帧仍有 158，命中 5/6。策略已可运行但仍低于 predicted，
  默认不切换，MPC 继续作为专项调参项。
- 主仓新增 `./scripts/run.sh --strategy=mpc` 运行时覆盖，不修改默认 YAML；启动
  审计确认 `Pipeline ready: control_strategy=mpc`，可与 Webots
  `WEBOTS_GIMBAL_COMMAND_MODE=mpc_state` 直接配对回归。
- **响应延迟补偿**：Webots 注入 yaw=80ms、pitch=50ms。predicted 基线与
  `controller.delay.control_latency_s=0.08` A/B 的前 250 帧分别为
  TRACKING/TEMP_LOST=`229/18` 与 `232/15`（轨迹由独立进程启动，不能作严格
  同样本 DPS 结论）。补偿档诊断新增字段显示
  `control_latency_s=0.08`、`double_compensation_risk=false`，总预测时间上限
  由约 0.145s 提升到约 0.223s，证明延迟项已进入瞄准控制目标；需用锁定随机种子
  和同步启动做最终命中率标定。
- **armor_pose 矩阵入口**：`run_difficulty_matrix.sh` 新增
  `HFUT_MATRIX_INPUT_MODE=armor_pose`，单格 stationary/L1 20s 冒烟约 570 帧，
  命中率约 0.847，确认矩阵脚本能切换到直接位姿链路；完整 12 格矩阵仍待长跑。

### PnP 系统偏差协方差闭环修复

启用默认系统偏差后的首轮 L1/L2 子矩阵出现周期性 `all_gate_fail`：stationary
L1/L2 分别 25/52 次，几乎每次换板一帧；拒止由 yaw chi2 主导。根因有两层：

1. V1 门控用固定 R，提交更新用 direct/PnP 元数据 R，评估与更新不一致；
2. 发布器只把零均值噪声写入元数据，边缘板最高约 0.2-0.3rad 的系统 yaw
   偏差仍被当作高置信度观测，中心 yaw 被拉偏后，下一块正面板产生突变创新。

修复：V1 单/双板门控和更新统一使用 `measurement_covariance`，所有分量保留
配置噪声地板；发布器逐板协方差用 `hypot(random_std, abs(systematic_bias))`
覆盖当前注入的 yaw 与沿光线测距偏差。协议 v3 不变。

22s 回归结果：

- stationary L1：624/624 有效更新 commit，TEMP_LOST=0，命中率 98.1%；
- stationary L2：628/628 commit，TEMP_LOST=0，命中率 90.1%；
- random L2：591/591 commit，TEMP_LOST=0；另有 37 帧 direct 空帧，命中率
  94.1%，未见扩大边缘板协方差导致的机动响应退化。

结论：任务 2 的 PnP 风格偏差现在既能产生高角度失真，也不会因为不一致/过小
协方差人为击穿任务 1 的 tracker 门控。

### 2026-07-25：vision 抖动致虚假整车平移调查（video_test，任务 1）

**现象**：实片 `test_video/output.avi`（100fps、目标约 1.5m、原地变向变速旋转）
经 `run_video_test.sh` 复现：静止目标上估计速度 |v| 中位 0.47 / p90 1.00 /
最大 1.80 m/s，估计中心跨度 0.39×0.26m；831 帧有效状态、820 帧 TRACKING。
视频无真值，所有指标为相对量。

**测量层事实**（对角点/PnP 输出的离线复算）：

- 同一 2D track 的板位置测量噪声 p50 3.6cm / p90 11.6cm / p99 20cm（每帧，
  含真实板切向运动约 2cm/帧）；运动模糊 + 小板（约 100px 宽）所致。
- 单帧重建中心（板位 − r·朝向）std 仅 6.5/9cm，观测本身并不携带 0.4m 量级
  的系统性偏差；视角分箱偏差 ≤5cm。滤波器把零均值重尾噪声放大成了相干
  速度误差（Singer 加速度状态的记忆性使 |v| 以板周期锯齿积累）。
- IPPE 孪生解闪烁：141 个 yaw 跳变帧全部为重投影近简并分支切换
  （两解误差比 0.81-0.98），yaw 方波 ±0.85rad；但这是零均值闪烁，不是
  平移的主要来源。

**已实测排除的干预**（均在 video_test 上比基线更差，故未保留）：

1. IPPE 分支时序滞回（锁上一帧分支）：持续旋转目标上错误分支自锁
   （镜像分支反向旋转、锁定不可逃逸），|v| 中位 0.81、中心跨度 2.34m。
2. 按歧义度膨胀 R_yaw（σ 地板 0.25-0.48rad，419/834 帧触发）：yaw 是近距
   旋转靶中心的锚定信息，降权后位置更新把中心沿圆拖动，|v| 中位 0.79、
   最大 7.5m/s。暂停旋转见证变体更差（中位 0.96）。
3. 平移创新一致性见证驱动自适应平移 Q（静态 0.15 地板）：带宽调制引起
   门控抖动与重置增多，transitions 23→56，|v| p90 2.15。
4. 旋转过程噪声 delta_rate 3.5→8.0：|v| p90 2.67，更差。
5. singer_sigma 5.0→2.0：|v| 中位降至 0.36 但中心跨度 1.16m、transitions
   40，净变差；且该参数是 random 靶 DPS 的关键（235>214>177），不能为
   静止场景过拟合。
6. 面板切换滞回（新增 `vehicle_tracker.hypothesis_selector.panel_switch_hysteresis`，
   0.3/0.8 两档）：面板轨迹确实变干净（49 次切换全为真实换板、无抖动），
   但 |v| p90 1.39/1.48 仍差于基线 1.00——面板抖动不是平移主因。机制保留、
   默认 0.0 关闭，配置内有评估记录。

**对照：webots vision 验证环境（有真值）基线**：stationary spin 6rad/s
60s，中心误差中位 4.1cm / p90 9.2cm / 最大 23.1cm；|v| 中位 0（死区）、
p90 0.12、仅 4% 帧 >0.5m/s。即验证环境内该现象轻微，视频场景
（10rad/s、1.5m、100fps 模糊）超出云台角速度能力与矩阵包络
（矩阵 spin≤8rad/s、距离 3-7m）。

**结论**：该场景下 |v|≈0.5m/s 接近"能跟 8m/s² 机动的滤波器在 9cm/100Hz
噪声下的稳态速度误差"的信息论下限，五种机制性干预均被数据否决。保留改动：
IPPE 歧义度等效 σ 上报链路（detector→Armor msg→ObservationData→诊断，
纯元数据不改变行为，为后续 InEKF 调参与歧义帧分析提供观测手段）+
滞回机制（关闭）。后续可行方向（未做）：旋转圆弧中心估计（旋转-only
的中心观测，不经平移动力学）；高帧率模糊输入的专用噪声 profile。

**验证**：`armor_pose_ippe` 单测新增真实闪烁帧歧义度回归（近简并帧
`ippe_yaw_ambiguity ≥ 0.2`）；ctest 13/13；最终态 video_test 指标与基线
逐项一致（滞回关闭、上报链路无行为影响）；v15 全矩阵回归见本节末尾
追加（若与基线净中性即定稿提交）。

**v15 矩阵回归（2026-07-25，16 格×60s，vision）**：总 eDPS 1421
（moving 185 / orbit 199 / random 440 / stationary 597），全格 QP% 100%、
fallback 0，TRK% 除 random_walk_L4 64%（已知近距/搜视边界）外均 ≥80%，
中心误差与转速估计均处历史正常区间。保留改动为纯元数据 + 默认关闭机制，
矩阵行为与基线一致，任务 1 定稿提交。

### 2026-07-25：4 板后端切换 InEKF 与对齐调优（任务 2）

**起点**：`backend_config.backend_type` 由 `ukf_v1` 切为 `inekf`。stock InEKF 与
ukf_v1 存在能力差距：无旋转见证/自适应旋转 Q、无 V1 的朝向/距离自适应
观测噪声、无 ±π yaw 折叠、dza 强制非负、后验 max_center_jump 0.25、
噪声走 ypd_ba_low_weight（与 V1 验证过的 R 模型不同）、运动 CA。
stock 冒烟 stationary_L1（30s）：TRK 89%、eDPS 80、ω 误差 -1.28。

**补齐改动（代码）**：

1. 旋转见证移植（noteArmorAngle/resetArmorAngleWitness/对称陷阱有界推移）
   与 predict() 自适应旋转 Q（见证 |ω| 驱动， warmup 全带宽）。
2. V1 观测噪声模型抽为共享头文件
   `trackers/vehicle/adaptive_measurement_noise.hpp`（距离缩放、朝向/距离
   自适应 R_yaw、BA 协方差地板、±π 折叠），V1 后端改为引用同一实现；
   新增 `V1StyleNoiseModel`，InEKF `noise_profile: "v1"` 启用；
   InEKF evaluate 单/双板 yaw 创新接入 ±π 折叠。
3. 后验与结构对齐 V1 已验证值：max_center_jump 0.30、有符号 dza
   [-0.15,0.15]（含 InEKF apply_state_constraints 去掉硬编码 min_dza=0）、
   dual_update 结构增益 0.25/0.0、slow_structure 先验 0.20/0.20。
4. 垂直通道与 dza 证据代码移植（vertical_dynamics_scale、VZ/AZ 限幅、
   noteDualHeightEvidence），**默认关闭**（见下方评估）。
5. 注意：矩阵脚本 `run_difficulty_matrix.sh` 默认 `HFUT_MATRIX_STRATEGY=mpc`，
   本轮所有矩阵数字均为 mpc 策略；ukf_v1 基线 1421 同为 mpc。

**矩阵结果（16 格×60s，mpc）**：

| 配置 | 总 eDPS | moving | orbit | random | stationary |
| --- | --- | --- | --- | --- | --- |
| ukf_v1 基线 (v15) | 1421 | 185 | 199 | 440 | 597 |
| InEKF 对齐版 (matrix_inekf_v1) | 1565 | 242 | 221 | 369 | 733 |

- 强项：stationary 全档（L1-L4 174/207/173/179 vs 164/177/146/110），
  低速档全面更好（moving_L1 136 vs 69、orbit_L1 126 vs 67、
  random_L1 185 vs 117）；stationary_L2/L3 中心误差 2.7/2.9cm、
  ω 误差 -0.09/-0.08 均优于基线。
- 弱项：高难度格 TRK% 连续性（moving_L3 67% vs 80%、moving_L4 45% vs
  83%、orbit_L4 61% vs 92%、random_L3 61% vs 96%、random_L4 41% vs
  64%），reconstruction/posterior 拒止明显多于 ukf_v1（random_L2：
  72+19 vs 17+17）；spin 6/8 + 平移时 ω 锁 0 为两后端共有弱点
  （ukf moving_L3 |w| 中位也仅 1.20）。

**单格 A/B 评估（注意 webots 噪声未固定种子，±15% 内不可作结论）**：

- motion_profile singer（random_L2/L3）：95/48 vs CA 118/53，回退 CA。
- vertical_dynamics_scale 0.02：random_L2 TRK 77→67%，判定有害，回退 1.0。
- dual_height_evidence_gain 0.15：zpp 33.5→7-11cm 明显改善，但 random 格
  eDPS 下降（118→83/94，含方差），moving_L2 改善也可能为方差；默认 0.0
  关闭，机制保留。
- V1 风噪声模型 vs FixedCartesian vs ypd_ba：v1 显著最优（FixedCartesian
  触发对称陷阱 ω 锁 0；ypd_ba TRK%/eDPS 均低）。

**待补**：定稿候选复跑确认（matrix_inekf_v2）与 predicted 策略对比，
结果见本节末尾追加。

**定稿关键调参（ca_process_noise_acc）**：InEKF 高难度格拒止率 ~11%（ukf_v1
~4%）的根因是 `inekf_runtime.ca_process_noise_acc` 继承全局 0.1，速度先验在
机动间塌陷、大创新被门控/后验拒止；此前 Singer 实验无效也因
`build_invariant_Q` 只按 CA 块填充（Singer σ 不参与）。调至 5.0（与 ukf_v1
验证过的 Singer σ=5.0 同量级）后 6 个弱格全部修复，5 个反超基线：
moving_L3/L4 54/33（基线 41/28，TRK 67/45%→97/95%，ω 5.97/7.19 解锁）、
orbit_L3/L4 67/57（35/36）、random_L3 115（107）、random_L4 31（60，仅剩
已知近距/搜视边界差距）。

**v3 全矩阵（16 格×60s，mpc，ca_acc=5.0）**：总 eDPS **1712 vs 基线 1421
（+20%）**；moving 287/185、orbit 268/199、random 570/440、stationary
587/597。TRK% 除 random_L4 65%（≈基线 64%）外全 ≥85%；ω 高档全解锁
（moving_L4 7.19、orbit_L4 7.93、stationary_L4 7.99，误差均 <1 rad/s）。
方差备注：单格 ±15%、总量 ±8% 级（webots 噪声未固定种子），即使下界仍
高于基线；stationary_L3/L4 本轮 135/98 略低于基线 146/110，此前两轮为
173/179、170/150，判为方差。

**predicted 策略最终判决（16 格×60s）**：ukf_v1+predicted 基线 **1470**
（moving 198 / orbit 248 / random 375 / stationary 649；mpc 基线为 1421，
mpc 仍略低于 predicted，与历史一致）。InEKF+predicted **1840（+25%）**：
moving 277 / orbit 322 / random 573 / stationary 668；16 格中 14 格优于
基线，random_L4 36 vs 40、stationary_L3 165 vs 210（该格四轮结果为
173/170/135/165，基线 146/210，判为方差交错）除外。
video_test 实片冒烟（InEKF）：|v| 中位 0.39 / p90 0.78 / 最大 1.73，
中心跨度 0.30×0.35m，均略优于 ukf_v1 基线（0.47/1.00/1.80、0.39×0.26）。
**结论：`backend_type: "inekf"` 定稿为默认 4 板后端**，ukf_v1 保留注册可随时切回。

### 2026-07-25：观测宽容度——歪预测器不再拒准观测

**问题**：预测器发散时，准确观测因大修正被后验/门控持续拒止，轨迹抱着
歪预测打歪靶直到 12 帧 reject-streak 重置。

**改动（三项）**：

1. **后验中心跳变限幅提交**（InEKF tryUpdateSingle/Dual）：中心修正超过
   `posterior_sanity.max_center_jump`（0.30m）时不再整帧拒止，截断到限幅
   提交（诊断 `_cjc` 标记）；yaw 跳变、结构、协方差检查维持拒止。
2. **TRACKING 持续冲突放宽门**（vehicle_tracker）：单帧严格门失败仍拒
   （明显异常才舍去），连续第 2 帧严格门失败但 init_relax 放宽门通过 →
   判定预测器发散，直接采信 top1 观测。
3. `reject_reset_streak_frames: 12 → 6`：重置自愈更快。

**验证**：predicted 全矩阵总 eDPS 1790 vs 改前 1840（-2.7%，在 ±8% 方差
内）；弱格拒止显著减少（moving_L4 33→11、random_L2 53→27），moving_L4
TRK 86→94% 且 ω 解锁（0→6.5 rad/s），moving 场景 314 vs 277；`_cjc`
实际触发稀少（多数冲突在门控层已由第 2 项吸收）。mpc 全矩阵结果：1740 vs 改前 1712（+1.6%，方差内偏正）；mpc moving_L4 ω 7.69（误差 -0.31，该格历史最佳），moving/orbit 高难度格 TRK 94-99%。两策略合计评估：总 eDPS 变化在方差内（predicted -2.7%、mpc +1.6%），弱格拒止减半、歪预测器拒准观测链路打通，定稿保留。

### 2026-07-25：random_walk 场景语义变更（小陀螺增强，仿真仓）

仿真仓 target_spinner 随机走位的小陀螺行为增强（**场景语义变更，此前的
random_walk 矩阵数字与之后不可直接比较**）：

- 进入小陀螺概率 25%→45%（`WEBOTS_TARGET_RANDOM_SPIN_PROBABILITY`，
  小陀螺后不连续小陀螺）；满速保持 3-6s（`SPIN_HOLD_MIN/MAX_S`，满速后
  才开始计时，最短 3 秒）；`SPIN_PAUSE_TRANSLATION` 默认 false（边转边移，
  置 true 则平移刹停原地旋转）。
- 顺带修复两个控制器 bug：①`uniform(lo,hi)` 重复叠加 lo，实际采样区间
  [2lo, lo+hi)，行为段/保持时间全部偏长约一个 lo（hold 实测 7.05/7.90s）；
  ②满速计时与退出分支互锁震荡（保持结束后"未达满速→拉回目标转速"与
  "减速回零"逐帧互切，转速卡在目标值附近永不退出），改为退出相锁存。
- 验证（60s×4 档真值分析）：L1-L4 小陀螺段 3-6 段/格、段长 3.4-5.9s 全部
  ≥3s（L1 两个 1.xs 段经平移速度核实为跟随转向而非小陀螺）、旋转期间
  平移 |v| 中位 0.00、小陀螺时间占比 23-36%（原约 10-15%）；矩阵
  random_walk 子集 TRK 99/94/98/78%，无崩溃。
- 改动文件（仿真仓，未提交）：`controllers/target_spinner/target_spinner.cpp`
  （+HFUT_DEBUG_SPIN 环境变量调试输出）、`config/difficulty_levels.env`、
  `run_random_spin_target_test.sh`、`README.md`；控制器二进制已重编译。

### 2026-07-25：真值误差口径调参战役（vision/armor_pose 双模式）

**评估口径**（用户指定）：以真值误差为主（ce/pe 当前/预测中心误差、ω 误差
与方向一致率、更新及时性 commit/拒止计数、TRK%），DPS 仅参考；测试覆盖
全部 4 场景×L1-L4（16 格×60s，predicted，新 random_walk 场景语义）。

**基线（InEKF，ca_acc=5.0）**：

- vision：ce_med 均值 8.0c（最差 random_L4 14.0c）、pe_med 均值 15.2c
  （pe/ce≈1.9）、|werr| 均值 1.67 rad/s、3 格 ω 锁 0；
- armor_pose：ω 全格完美（werr -0.05~-0.10、方向 ~100%），pe/ce≈1.3；
  静止格 ce=pe≈7-8c 为发布器注入的 PnP 风格系统失真底噪（非滤波问题）。

**归因**：①预测滞后（pe）是两模式共有短板→预测器/估计器侧；②vision ce
约为 armor_pose 两倍→视觉噪声链路；③vision 的 ω 锁 0 是视觉侧（orbit_L3
实测板被 FOV 边缘裁切，关键点 x=1440 贴边，属 FOV/搜视场景问题而非滤波）；
④random_L4 为已知近距检测/搜视场景问题。⑤预测外推已是 CA（加速度限幅
8 m/s²），pe 由速度/加速度估计误差主导。

**实验（均在 armor_pose 或 vision 子矩阵上 A/B，单格 ±15% 方差注意）**：

| 实验 | 改动 | 结果（pe_med 对比） | 结论 |
| --- | --- | --- | --- |
| E1 | ca_process_noise_acc 5.0→8.0 | 9 格总和 91.2 vs 96.3c（-5%，方差内偏正，难格 -0.6~-2.4c），ce 无代价 | **保留 8.0** |
| E2 | 输出平滑 vel cutoff 0.8→1.5 / beta 3→5 | 93.5 vs 91.2c，中性 | 回退 |
| E3 | 预测加速度限幅 8→12 m/s² | 4 格 pe 持平（加速度估计极少触限） | 回退（顺手修正注释 6.0→8.0） |
| E4 | vision obs_noise_scale 1.5→1.2 | random 改善但 orbit_L2 TRK 97→86%，净负 | 回退 1.5 |

**结论**：仅保留 ca_process_noise_acc 8.0；当前配置在真值误差口径下已处
局部最优，主要剩余差距（vision ce 2×、orbit ω 锁、random_L4）均为
视觉/搜视场景层问题，不是滤波参数可解。定稿 vision 全矩阵（ca 8.0）：
ce 7.7c vs 基线 8.0c、pe 15.1c vs 15.2c、|werr| 0.92 vs 1.67、TRK 94%
vs 95%——方差内略优无回归，ca 8.0 定稿保留。

### 2026-07-25：vision 角点精化（sp_vision 式 NN+传统灯条混合）

**动机**：vision ce 约为 armor_pose 两倍，PnP 输入的 NN 关键点在远距/模糊
下噪声大（webots 真值投影角点误差中位 ~21px）；实片 IPPE 孪生解因关键点
抖动逐帧闪烁。参考 `reference/sp_vision_25`（detector.cpp:122-231 的 NN 粗
定位+局部 ROI 传统灯条重拟合）。

**实现**（启用仓库已有的 RoiPcaCornerRefiner 死代码，原先未接线）：

- `ArmorDetectorNN` 内 2D tracker 关联后，对每个检测在局部 ROI 内重跑
  灯条检测（OTSU 自适应阈值 + minAreaRect + 几何校验），通过才替换关键点；
  失败保留 NN 关键点，极端光照/模糊/遮挡下行为与未开启完全一致（跨环境
  泛化设计，非单一环境拟合）。
- **灯条内陷补偿**：传统拟合测的是灯条而非板轮廓，webots mesh 灯条内陷
  （实测宽度仅为真值投影的 0.944），直接替换会高估深度（depth_ratio
  1.022，静止格 ce 2.4-2.9→6.5-6.6）。改为按 2D track 维护
  NN板幅/精化板幅 比值 EMA 缩放精化角点——精化保形状稳定性、NN 保物理
  板幅，无环境常数。
- 配置 `detector.yaml corner_refine`（enabled、method=full_lightbar_roi、
  OTSU、几何约束）；`config_loader` 补 corner_refine 加载（原先缺失）。

**实片（video_test）效果**：IPPE 歧义度中位 0.251→**0.000**（孪生解闪烁
消除），跟踪状态切换 23→**3**，中心跨度 y 0.26→0.16m，pose_jump 持平，
重投影误差 0.128→0.590（minAreaRect 矩形约束 vs 透视梯形，亚像素级，
精化正则化换稳定性的预期代价）。

**webots 子矩阵（predicted，8 格 vs 同配置基线）**：moving/random 各格
pe 普遍改善 -1~-4c（moving_L2 13.9→10.5、orbit_L2 18.5→14.7 等），
静止格 ce 仍有 ~2.4c 残余回归（深度残差 1.012 vs 0.989，webots 渲染
特性——内陷辉光条 + 0.9635 缩放按 NN 标定，非真实板预期行为）。
全矩阵结果见追加。

**全矩阵定稿（16 格×60s，predicted，vs 同配置基线 tune_final_v）**：
pe_med 均值 **13.0c vs 15.1c（-14%）**、|werr| **0.69 vs 0.92**、TRK 96%
vs 94%、ce 7.6c vs 7.7c 持平；pe 改善覆盖全部场景（moving_L1 17.9→11.3、
moving_L2 13.9→10.1、orbit_L2 18.5→14.5、random_L2 16.2→14.4）。残余：
stationary_L2/L3 ce 5.4/4.6 vs 2.9/2.4（webots 内陷辉光条渲染特性，真实
板灯条即板边缘，不预期在实机出现）；stationary_L4 本轮 ω 锁 0（该格多轮
间翻转的已知边界）。实机判据：真实视频（video_test）的稳定性收益
（歧义 0、切换 3 次）与 webots 残余回归的成因不同源，保留定稿。

### 2026-07-30：outpost 跟踪链修复（armor_pose 模式）

**场景**：仿真仓新前哨战靶（3 板互成 120°、半径 0.275m、高度
1.618/1.516/1.414m、斜向下 15°、5m、±2.513 rad/s 匀速，旋向随机）。

**根因链（按发现顺序）**：

1. **初始中心镜像 bug（总根因）**：`OutpostInEKFBackend::initialize_state`
   按视觉 PnP 法线约定把 `obs.yaw` 减 π 再反推中心，但 armor_pose 直发
   记录经 `pipeline.cpp:387` 给出的是**真 radial yaw**（center→armor），
   减 π 后初始中心被镜像到真实中心另一侧 2·r=0.55m 处。此后所有
   "中心漂移/锁错面板/vyaw 翻号"都是这个起点的下游现象。
2. **(center, yaw, panel) 流形歧义**：单板 3D 观测下自由中心会吸收旋转
   形成"中心绕圈 vyaw 锁 0"或错面板镜像不动点。修复为**静态锚点**：
   中心冻结在 reset 锚点（initialize 与面板无关、精度 ~3cm），平移/速度/
   加速度状态钉零、K 的对应行置零，旋转全部交给 yaw/vyaw 拟合。
3. **观测噪声诚信**：outpost 后端原来用固定 0.02m R，忽略直发记录逐板
   协方差（5m 处视角偏差噪声达 0.29m）→ 门控全拒。改为记录协方差+
   配置地板，并按观测俯仰把记录噪声投影到 xy/z（z 通道保持锐利以分辨
   0.102m 板高差）。
4. **旋向镜像**：vyaw 反复翻号。加装甲方位见证（直发记录 radial_yaw
   差分）+ 镜像重锚（见证与 vyaw 持续反向 5 帧则从当前观测重置并以见证
   转速播种）。
5. **门限链过严**（死锁："进不了 STRUCTURED→无法提交→vyaw 学不出"）：
   模式入门 conf/margin 0.70/1.50→0.35/0.30、stable 5→2、出门 0.25/0.15、
   提交门 0.50/1.00→0.35/0.20、warmup 0.65/1.2→0.4/0.3；phase_audit
   dz_gate 0.035→0.10（适配视角偏差噪声）；后验中心跳变改为限幅提交。
6. **结构先验可适应**：半径与各面板 z_offset 由提交的观测 EMA 学习，
   钳制在先验 ±5%（用户要求：几何/转速是先验，场上允许 ~5% 偏差）。
   靶高修正 1.818→1.618（z_offset_0 回到 +0.102）。

**最终验证（全新仿真 65s，armor_pose）**：TRACKING 1304/1697 帧、LOST 14；
中心误差中位 0.030m；|vyaw| 中位 2.634（真值 2.513）；板位误差中位
0.038m；**628 发 213 中、命中率 34.9%、DPS 62.7**（修复前命中率 ~0.3%）。
瞄准残差中位 2.4° 主要来自 0.2s 预测时域×2.5rad/s 相位运动，属预测/
弹道层而非跟踪层。vision 路径的 outpost（gestalt）未在本次范围。
