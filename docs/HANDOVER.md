# 项目交接文档（HANDOVER）

> 目的：让接手方（人或 AI）在**不依赖原对话**的情况下，完整掌握当前项目
> 的工作空间、进度、验证方法与未决问题。最后更新：2026-07-26。
>
> **读我顺序**：§1 仓库地图 → §2 怎么跑 → §3 进度总览 → §4 工作区状态 →
> §5 配置要点 → §6 未决问题 → §7 下一步建议 → §8 环境坑 →
> §9 持续工作日志。

---

## 1. 工作空间地图

| 路径 | 角色 | 说明 |
| --- | --- | --- |
| `/root/hfut_auto_aim` | **主仓（全部新改动在此）** | ROS-free 自瞄管线（检测/跟踪/选板/控制/开火）。源自 hfut_rm_auto_aim_ws 的移植改造，现已完全独立。 |
| `/root/hfut_auto_aim_sim` | **仿真仓** | Webots 闭环仿真：目标车（旋转/横移/随机走位）、相机桥、得分系统。与主仓通过 `/tmp/hfut_auto_aim_webots/` 下的原子文件交换数据。 |
| `/root/hfut_rm_auto_aim_ws` | 参考仓（**勿改**） | 原 ROS2 版自瞄。只读参考；主仓已与其解耦，各自使用独立配置文件。 |
| `/root/reference/sp_vision_25` | 参考算法 | 上交 sp_vision_25 自瞄，机制借鉴来源（朝向自适应 R、NIS 滑窗重置、来/离角选板等）。 |
| `/root/reference/sp_vision_25_gestalt_system_bridge` | gestalt 桥参考 | gestalt 仿真器对接参考与其手册（参数表已整理进主仓 docs/GESTALT_PARAMS.md）。 |
| `/root/code_for_test` | 辅助脚本 | 采样/对比小工具（真值对比、抖动采样等），多为一次性分析。 |
| `/root/qpOASES` | 依赖 | MPC 的 QP 求解器，已安装到 `/usr/local`（libqpOASES.so.3.2）。 |

**数据流（仿真闭环）**：
```
Webots 目标+相机 ─ camera_frame.bin ─► hfut_auto_aim (bringup_sim)
                 ─ armor_pose_frame.bin (armor_pose 模式) ─►
hfut_auto_aim ─ gimbal_command.bin ─► Webots 云台 + 计分 ─ score.txt
真值：/tmp/hfut_auto_aim_webots/target_truth.jsonl（地面系，z 比 shooter 系高 0.405m）
诊断：run.sh --diagnostics=<path>.jsonl
```

## 2. 怎么跑

### 主仓构建与测试
```bash
cd /root/hfut_auto_aim
./scripts/build.sh                         # CMake 全量构建（产出 build/bringup_sim）
ctest --test-dir build --output-on-failure # 13 个单测，应 100% 通过
```

### 仿真单跑（手动）
```bash
# 终端1：起仿真（vision 输出；可换 run_moving/random_spin_target_test.sh）
cd /root/hfut_auto_aim_sim
WEBOTS_TARGET_SPIN_RATE=6.0 WEBOTS_AUTO_AIM_OUTPUT_MODE=vision \
  ./run_stationary_spin_target_test.sh --batch --minimize
# 终端2：等相机流新鲜后跑算法（60s，写诊断）
cd /root/hfut_auto_aim
timeout 60 ./scripts/run.sh --input-mode=vision --diagnostics=/tmp/run.jsonl
# 调试截图：HFUT_DEBUG_SNAPSHOT=/tmp/snap.png 加 --debug
# 观测打印：HFUT_DEBUG_OBS=1；转速/见证打印：HFUT_DEBUG_OMEGA_EMA=1
```

### 难度矩阵（标准回归手段）
```bash
cd /root/hfut_auto_aim_sim
./run_difficulty_matrix.sh /tmp/matrix_vX 60            # 3场景×L1-L4 共12格
python3 tools/matrix_report.py /tmp/matrix_vX           # 汇总表
# 子集：./run_difficulty_matrix.sh /tmp/m 60 "random_walk" "L3 L4"
```
- 难度定义：`config/difficulty_levels.env`（spin 2/4/6/8 rad/s、横移
  0.5/1.0/1.5/2.0 m/s、加速度 1.5/3/5/8 m/s²）。
- 矩阵每格自动重启仿真、等真值+相机双流、帧数不足重试一次、保存
  诊断/真值/score。
- **调参纪律：单场景数字会过拟合；任何参数改动以全矩阵回归为准。**
  有效 DPS 口径 = hits×20/算法秒数（score.txt 的 dps 字段被空闲期稀释）。

### 关键文档
| 文档 | 内容 |
| --- | --- |
| `docs/VALIDATION.md` | **全部验证记录与根因分析（最重要的历史档案）**，按时间顺序追加（最新在末尾） |
| `docs/GESTALT_PARAMS.md` | gestalt 桥全参数表（实体 ID、队伍、延迟、坐标系等） |
| `docs/GESTALT_BRIDGE.md` | gestalt 桥接实现说明 |
| `docs/GESTALT_WSL2_TUTORIAL.md` | gestalt 环境教程 |
| `docs/UNMIGRATED_FEATURES.md` | ROS 仓尚未迁移的功能清单 |
| `hfut_auto_aim_sim/README.md` | 仿真场景、输入模式、难度矩阵用法 |

## 3. 工作进度总览

### 已完成（全部经过仿真验证，细节见 VALIDATION.md）

1. **移植与规范化**：传统灯条检测器回退链路（detector.impl=nn/traditional/auto）；
   实验版命名规范化（norm4_v3→trackers/vehicle，DELTA→YAW 等）；配置文件按
   主题拆分（gimbal_pipeline 总控 / simulation / detector / tracker /
   controller），tracker→controller→master 深合并。
2. **跟踪防抖与预测修正**：提交状态透出、motion guard 协方差同步、yaw rate
   guard、选板滞回、CA 外推限幅、实测处理延迟计入预测、OneEuro 速度自适应。
3. **功能**：selector.blocked_robot_ids（outpost 屏蔽，含回退路径）；
   selector.max_distance=6.0 距离门；TEMP_LOST 自适应惯性外推；
   roll/pitch 整车姿态估计器（body_attitude_estimator.hpp，已发布
   center_pose+可视化+诊断）。
4. **难度矩阵与全局调优（vision）**：
   - **对称板陷阱修复（核心）**：静止 yaw 模型+全同装甲板=观测总能匹配，
     ω 锁 0 不自愈。新增**无模型方位见证**（观测板绕中心方位角差分，
     ±90°换板帧剔除），持续背离>2 rad/s 时有界推移 yaw_rate；自适应旋转
     Q 的驱动从 center-yaw EMA 切换到该见证（静止档 ω 尾巴 p90 3.79→0.51）。
   - 拒止重置按原因分流：重建拒止占多数→标称结构重播，NIS 拒止→保留
     已学结构。换板伪迹 |inst|>12 rad/s 跳过。空转保持
     `controller.idle_hold_s: 2.0`。
   - **结果**：转速全档收敛（2/4/6/8 → 1.97/4.50/6.63/9.03），总 eDPS
     1455→1550。
5. **远距清除修复（第二批）**：检测 ID 误读按空间归并
   （`tracking.id_association_max_distance_m: 0.6`）；位置观测噪声随距离
   缩放（横向 ×r/2、纵深 ×(r/2)²）、R_yaw 3m 起二次放大（≤25x）、±π yaw
   折叠、重捕获门限放宽 `ukf_v1.gate.init_relax: 3.0`、检测 conf 0.5→0.35。
   4-5m TRACKING 23%→48%、5-6m 9%→24%，标准矩阵无回归。
6. **4 板后端切换 InEKF（2026-07-25，本提交）**：`backend_type: "inekf"`。
   补齐旋转见证+自适应旋转 Q、V1 风朝向/距离自适应观测噪声（抽共享头文件
   两后端复用）、±π 折叠、有符号 dza、后验门限对齐；关键调参
   `inekf_runtime.ca_process_noise_acc: 5.0`（继承 0.1 塌陷速度先验致高难度
   格拒止风暴）。矩阵（16 格×60s）：mpc 1712 vs ukf_v1 1421（+20%）、
   predicted 1840 vs 1470（+25%）；高档 ω 全解锁，TRK% 无系统性弱格。
   ukf_v1 保留注册可切回。
7. **选板修复**：转速自适应朝向门——|v_yaw| 0.5~3.0 rad/s 插值，低速
   40/60°、高速 65/85°（`solver.facing_enter/exit_low_spin_angle`）。
   低速错板率 39.5%→18.0%，高速档 eDPS 回到基线。
8. **仿真仓**：bridge 指令序号重同步（pipeline 重启不再冻结云台）；
   难度层级预设 + 矩阵运行器 + 报告脚本。

### 已核查但未启用
- **MPC / 轨迹优化器**：代码已完整迁移（与 ROS 仓逐字节一致，qpOASES
  已链接，已注册，`controller.strategy: "mpc"` 可选）。冒烟实测修复
  `mpc.vel_clamp.max_linear_speed 1.0→3.0` 与 `mpc.N 50→16` 后 TRACKING
  59%→77%，仍低于 predicted 的 96%（指令 yaw 误差中位 5°，开火基本不
  触发）。**当前 strategy 保持 "predicted"**，MPC 待专项调参（见 §6）。

## 4. 当前工作区状态

### /root/hfut_auto_aim
- `main` 与 `origin/main` 同步。2026-07-25 新增两个提交：`a8a83e8`（任务 1：
  IPPE 歧义度上报 + vision 抖动虚假平移调查，详见 §9 与 VALIDATION.md）与
  任务 2 提交（4 板后端切换 InEKF，predicted 矩阵 1840 vs ukf_v1 1470）。
- `f9f3aa3` 之后至任务提交前的 5 次提交：`a24aa57`（random 靶 armor_pose
  调优）、`70258f9`（状态估计与 MPC 仿真控制）、`fc3a05c`（Vision IPPE
  重投影选解 + 检测器 track_id 接通）、`6681bc8`（真实视频离线测试模式
  video_test）、`48c9f14`（3D 可视化 v1，无法正常工作，见 §6 第 8 条）。
- **验证基线：单测 13 个，2026-07-25 现场复跑 ctest 13/13 通过；难度矩阵
  已扩为 4 场景×4 档共 16 格（含 orbit_spin），矩阵脚本默认
  `HFUT_MATRIX_STRATEGY=mpc`，跑 predicted 需显式指定。**
  受限沙箱内 `gestalt_bridge_client` 可能因 `socket()` 被拒而失败，须在
  允许本地 socket 的环境运行后再判断回归。

### /root/hfut_auto_aim_sim
- `ros-free` 分支仍有未提交改动（2026-07-20 现场核对）：7 个已跟踪文件修改、
  3 个未跟踪文件；不要在未检查前清理或覆盖。
- bridge：指令 seq 回退重同步；`armor_pose` 协议 v3 输出完整板面四元数，
  `radial_yaw` 改为独立按四板均值中心计算；对应 controller 二进制已重编译。
- 倾斜夹具：新增 `WEBOTS_ARMOR_LAYOUT_ROTATION`，world/template 将四板包进
  可旋转布局 Transform，用于验证布局面倾斜和世界 Z 旋转轴不垂直布局面。
- 回归工具：新增 `config/difficulty_levels.env`、`run_difficulty_matrix.sh`、
  `tools/matrix_report.py`；README 同步难度矩阵、协议 v3 和倾斜夹具说明。
- 修改文件：`README.md`、`config/target_robot.env`、bridge 源码/二进制、
  `run_stationary_spin_target_test.sh`、两个 world 文件；上述全部仍未提交。

## 5. 配置要点（当前生效值）

- 总控默认：`bridge_path: webots`、`input_mode: armor_pose`、
  `detector_impl: auto`。难度矩阵会显式覆盖为 vision，不能把矩阵结果当成
  默认直接位姿模式的结果。
- 跟踪器：`VehicleArmorTracker` + **`InvariantPoseBackend`（InEKF，默认）**；
  ukf_v1 保留注册（`backend_type: "ukf_v1"` 可切回）。InEKF 关键配置：
  `noise_profile: "v1"`（V1 风朝向/距离自适应 R）、`motion_profile: "ca"`、
  `inekf_runtime.ca_process_noise_acc: 8.0`（防速度先验塌陷；8.0 经真值
  误差口径 A/B 对难格 pe 小幅改善，见 §9 调参战役条目）、
  旋转见证+自适应旋转 Q（EMA alpha 0.10）、有符号 dza [-0.15,0.15]、
  后验 max_center_jump 0.30；vision 观测噪声 scale 1.5（armor_pose 0.35）；
  `reject_reset_streak_frames 6`（12→6：歪预测器更快用观测自愈）；
  `gate.init_relax 3.0`；观测宽容度：后验中心跳变限幅提交（不再整帧拒止）
  + TRACKING 持续冲突放宽门（连续 2 帧严格门失败但放宽门通过即采信观测）；
  `attitude.mount_pitch_deg +15.0`（符号自校准）、`attitude.apply_to_geometry true`
  （整车 roll/pitch 姿态并入几何，可视化与预测跟随安装面倾斜）。
- 控制：`strategy: predicted`；`prediction_extra_s 0.05`；开火窗 0.1；
  选板 `min_movement_with_facing` + 转速自适应朝向门（40/60↔65/85）+
  `switch_movement_margin 0.005`；`idle_hold_s 2.0`。
- 选择器：`priority_list`；`blocked_robot_ids: ["outpost"]`；
  `max_distance 6.0`。
- 检测：NN（onnxruntime CUDA EP）`conf_threshold 0.35`；
  `simulation.yaml bridge.webots.keypoint_scale: 0.9635`（webots PnP 深度
  偏差补偿）。
- MPC（备选未启用）：`N 16 dt 0.05`、`vel_clamp.max_linear_speed 3.0`。
- Gestalt 当前源码配置：idle scan 为等待 0.5s、yaw 120°/s、pitch 30°/s、
  pitch ±10°；旧 README/BRIDGE/TUTORIAL 中 3s、60°/s、20°/s 的说明已滞后。

## 6. 未决问题（按优先级；算法问题详见 VALIDATION.md）

1. **Gestalt 开火安全默认值（下次 Gestalt 运行前先处理）**：
   `scripts/run_gestalt_stdio.sh` 当前无条件追加 `--allow-fire`，launcher 中追加
   `--no-fire` 的逻辑被注释，因此实际默认允许开火。**文档漂移已修复**
   （2026-07-25：README、TUTORIAL、BRIDGE 已同步为实际行为与实际
   idle scan 值 0.5s/120°/30°、协议 v3 128 B record、sentry `66000005`）；
   代码的安全默认值（是否恢复默认禁火）仍需在下次 Gestalt 实测前明确决定。
2. **random_walk L4（2m/s+8m/s²+7rad/s）TRACKING ~50%**：长黑屏由近距
   <1.2m 检测盲区（约一半）与高速换向逃逸（约一半）构成；属检测/搜视
   层面的场景级问题，不是滤波问题。
3. **MPC 未达可用线**：瞄准中值误差 5°（需压到 1-2°）。方向：参考轨迹
   时域（振荡目标 CA 发散）、延时标定、开火协调。gestalt 弱云台场景的
   理论优势（QP 约束内可行轨迹）成立，调通后再评估切换。
4. **高速档转速过冲 ~10%**（方位见证 EMA 系统性偏快）：影响 spin 6/8
   命中率约 5-10 个点。InEKF 切换后高档 ω 误差已显著收敛（多数格
   <|1| rad/s），此项改为继续观察。
5. **移动靶预测残差 14-20cm**：由速度估计滞后主导，prediction_extra_s
   全局不敏感（0.12 明确有害）；非时域参数可解。
6. **已知物理边界**（不修，记录）：25° FOV（垂直 ±7°）高速逼近 ~1.5m
   检测黑屏死锁；<1m 大板失效；webots 暗色 mesh 上远距（>5m）NN 检出率
   低（4-5m ~90%、6-7m ~25%）。
7. **静止档 ω 尾巴残留** p90 0.5 rad/s（已大幅改善，未归零）。
8. **3D 可视化调试器无法正常工作（待修理，优先级低）**：`tools/debug_3d/`
   浏览器端 Three.js 三维回放工具（提交 `48c9f14`，提交信息自述
   “存在问题”）。用法见主仓 README（`./scripts/run_debug_3d.sh`）。修理前
   不要依赖其显示结果下任何算法结论。
9. **gestalt 专属待办**：串口/相机实机通信未接（仍在仿真阶段）；gestalt
   端第一视角归属（红/蓝哨兵）切换——手册见 GESTALT_PARAMS.md。

## 7. 下一步建议（接手方参考）

1. 下次 Gestalt 运行前先统一禁火默认值与实际 idle scan/身份/协议文档，
   不要依据当前教程中的“默认禁止开火”操作。
2. 用户检查仿真仓未提交的协议 v3、倾斜夹具、矩阵工具和重编译二进制后 commit
   （完整清单见 §4）；主仓算法基线已推进到 2026-07-25 的任务 1/2 提交
   （InEKF 为默认 4 板后端）。
3. 若要继续提 DPS：优先 random_walk L4 的近距离检测与搜视策略，而不是
   继续调滤波参数（矩阵证据表明滤波已不是瓶颈）。
4. 若要用 MPC：单开任务做延时标定+参考轨迹调优，用难度矩阵回归，
   达标后再在 gestalt 上评估。
5. 出仿真前：用 `run_difficulty_matrix.sh` 做最终回归并附矩阵表到
   VALIDATION.md。

## 8. 环境坑（踩过，别再踩）

- **仿真是长跑进程但会自己退出**（原因未查明，现象是 webots 消失）：
  跑批前确认 `ps -eo pid,cmd | grep webots-bin` 在跑；pipeline 日志出现
  连续 `no camera frame for 2s` 即仿真已死。
- **kill webots 用 `kill -9 <pid>`**；本环境里 `pkill -f webots` 会把
  调用方 shell 一起带走（exit -1）。
- **真值是地面系，算法世界是 shooter 原点**：对比时减
  shooter_position（z 差 0.405m）。
- 矩阵每格会删 `/tmp/hfut_auto_aim_webots/*` 并重启仿真；手动跑批别与
  矩阵并行（GPU/CPU 争用会产生 60-200ms 处理毛刺，足以让跟踪崩溃）。
- pipeline 重启指令 seq 从 0 重计——bridge 已做回退重同步，若换别的
  bridge 需注意同款问题。
- **`configs/gimbal_pipeline.yaml` 的 `global.bridge_path/input_mode` 是共享
  状态**：手动改为 gestalt 后，难度矩阵的算法端会去等 gestalt 相机流，
  全程 "no camera frame" 且无显式报错，表现为矩阵全格 0 帧。跑矩阵前
  确认其为 `webots`（gestalt 用完请改回）。
- 主仓与仿真仓的 `armor_pose` 必须同时使用协议 v3（184 B header + N×128 B
  record）；仿真仓已重编译但未提交的 bridge 二进制不能与旧源码/旧主仓混用。
- `gestalt_bridge_client` 单测创建 `127.0.0.1` 回环 socket；受限沙箱里会在
  `socket()` 处以 exit 1 失败，应在允许本地 socket 的环境复跑后再判断回归。
- GitHub 当前网络连不通，别尝试 push/fetch。

## 9. 持续工作日志

> 维护规则：从本节开始，后续每项实质工作都按时间倒序追加。至少记录
> 任务/目标、实际改动或调查、验证结果、剩余问题；详细实验数据仍写入
> `docs/VALIDATION.md`，本节写索引和结论。

### 2026-08-01：自瞄轨迹规划器——hero/main 合并统一（已完成）

- 方案（用户裁定，参考 `~/reference/sp_vision_25` readme §4"轨迹视角下的
  自瞄理论"及其 `tasks/auto_aim/planner` 实现）：**云台轨迹与射击轨迹
  （目标轨迹提前飞行时间）的重合度决定自瞄效果**，用轨迹规划器统一
  不同兵种的瞄准决策，兵种适配只需配置云台角加速度上限。
- **分支合并**：主仓 main fast-forward 合入 hero（此前 hero 分支的
  outpost 后端改进、弹道/阻力、计分器修复全部进入 main）；仿真仓
  ros-free 同理合入。此后单一 main 线 + 配置档区分兵种
  （`configs/` 步兵、`configs_hero/` hero）。
- **规划器**（`gimbal_controller/aim_trajectory_planner.{hpp,cpp}`，
  挂在 predicted 策略内，`controller.aim_planner.*` 配置）：
  - 参考=射击轨迹：1s 窗口（2×25×20ms）内每步取相对当前云台角最近
    的装甲板做弹道解算，换板在参考中形成突变三角波；
  - 双积分器 QP（决策变量=角加速度序列，box 约束=角加速度上限，
    qpOASES 热启动 ~1ms/轴），突变被平滑为可跟随的过渡段
    （**平滑≠滞后**：跟随段与射击轨迹完全重合）；
  - 输出当前时刻规划角+速度/加速度前馈（写入 cmd.yaw_v/yaw_a 等，
    供下位机 mpc_state 跟随）；开火仍由开火引擎按"准星 vs 板到达
    位置"门控，过渡段轨迹分离时自然关断（=sp 的偏离停火原则）。
- **清理**：删除交叉保持（crossing_hold/lead）、hero 特调开火概率
  参数等过渡性 hack；`min_shooting_angle` 配置化保留（默认 1.0°）。
- 配置档：`configs/` aim_planner acc 80/80 rad/s²（对齐步兵仿真云台），
  `configs_hero/` acc 12/12 + 弹道 12m/s + resistance 0.0175 +
  开火窗口 0.05m/0.2°（低弹频下"窗口即脱靶量"）。
- 验证（hero 1/8s 弹频 outpost）：轨迹平滑（换板过渡段连续、前馈
  合理），默认窗口 16.7%、紧窗口(0.05m/0.2°)19.4%、加开火防抖(8帧)
  11.8%——均低于交叉保持时代的 44% 峰值，**会话方差（估计翻转
  churn）仍是主要限制且与规划器无关**：稳定段命中，翻转段整段脱靶。
  步兵回归：规划器 ON/OFF 对照 23.3%/22.7%（同会话，无回归）；
  步兵 35-40% 峰值会话同样由 churn 主导。

### 2026-08-01：HERO 适配——12m/s 弹道+交叉保持开火（双分支落地）

- 任务（仿真仓 hero 分支）：弹速 12 m/s + 室内理想空气阻力；重云台禁
  高频振动 → 平滑少突变瞄准轨迹；弹频低但不固定，自瞄不含弹频——
  上位机只出“当前发射能否命中”的门控，能否发射由下位机 CD 决定；
  测试按 1 发/8 秒，以命中率为主。基线参数已记录于仿真仓
  `docs/hero_branch.md`。
- **分支结构（用户裁定）**：hero 涉及实现代码改动 → 两仓均保留
  main(步兵)/hero 双分支。主仓 main 止于 3d861a9（task1 通用改进）；
  全部 hero 内容在 hero 分支（c45273c → 43eb855）。
- 仿真侧（hero 分支，`573aa81`、`0008686`）：计分器弹道加二次阻力
  （`WEBOTS_BULLET_DRAG_K`，默认 0，模型与算法 solver 一致 a=-k|v|v）；
  `run_hero_outpost_test.sh`：弹速 12、k=0.0175、弹频 0.125Hz、云台
  速率 4rad/s、加速度 12rad/s²；计分器 seq 回退修复（跨运行不发射
  根因）、逐发日志 shots.jsonl、陈旧命令新鲜度门（0.5s）。
- 算法侧（主仓 hero 分支 `configs_hero/`，`--config-dir` 选择）：
  弹速 12、`solver.resistance: 0.0175`、`solver.outpost_crossing_hold`。
- **交叉保持（crossing-hold）**：三板转子不追板——瞄准转子近端环点
  （相对相机最正对的穿越点、中板高度）的**静态世界点**。云台近似
  静止（实测 gimbal_lag≈0，平滑少突变直接成立）；开火引擎在板穿越
  准星时自动开门（候选解算自带飞行时间提前量；高/低板因俯仰窗口
  1.24°>1° 自然被拒绝，无需实时跟踪板高状态）。MPC 策略参考提前量
  仅 50ms 不适用 12m/s（飞行 0.45s），故用 predicted+交叉保持。
- 关键实测结论：①开火窗口即脱靶量——门控在窗口早边打开即发射，
  弹着系统性偏一个窗口宽度，必须收紧窗口（0.05m/0.2° 生效版）；
  ②`min_shooting_angle` 默认 1.0° 会盖过收紧的窗口（已配置化）；
  ③瞄准点提前量在几何上无效（门控相对瞄准点开门，偏移自消）；
  ④模型相位存在 ~0.1s 当量的传输+滤波滞后（半转速实验证实），
  是低弹速下的主要残差来源。
- 验证（hero 1/8s 弹频，320s/轮）：最佳 44.4%（27发12中）、40.7%、
  28.6%——**会话方差大**：换板期提交周期性触发 TEMP_LOST 翻转
  （0.6-0.75s 一次），翻转风暴时整轮 0 中。当前主要限制。
- 排障记录：hero 初次零发射为三层叠加——MPC 策略提前量不适用、
  云台滞后>窗口、仿真计分进程旧会话残留。

### 2026-08-01：outpost 命中率提升——锚点外置+见证测速（已完成）

- 任务：提升 outpost 命中率（基线 34.9%/DPS 62.7），DPS 不能降太多。
- **验证口径修正（先做）**：①仿真脚本启动时补删 `target_truth.jsonl`/
  `score.txt`（此前旧会话残留真值污染分析，"预测板误差 3.18m"是伪信号）；
  ②算法端 webots 弹速 24.3→**22.5**（仿真仓 `WEBOTS_BULLET_SPEED` 实发
  22.5，此前 5m 处 pitch 系统误差 ~0.4°）；③score.txt 计数在计分器内存
  中累计，必须跑前跑后取差值；运行中删除 truth 文件无效（fd 指向已删除
  inode），只能重启仿真。新分析工具：`tools/analyze_outpost_run.py`。
- **根因链（逐层实锤）**：
  1. 冻结锚点=单帧观测反推（σ≈0.18m），锚点误差全程固化 → 同代码命中
     率 27%~35% 大波动；
  2. 用滤波自身 yaw 反推锚点无效：单板观测 (center,yaw,panel) 流形歧义
     使滤波围绕任意中心自洽，提交残差里**没有中心信号**；
  3. 直发记录有视角系统偏差（射程拉远 10%+yaw 拉向相机方位，均指向
     远离相机/+x），緣边记录协方差诚实所以能过门，会把任何用它们的
     估计器拉偏 +0.25m；
  4. vyaw 被緣边提交污染（-3.2 vs 真值 -2.513），见证差分 EMA 也被
     记录 yaw 偏差带快 ~3%；yaw_acc 残差成为恒定速差泄漏。
- **修复（全部在主仓 outpost_v3）**：
  1. **锚点外置**：`noteAnchorObservation` 对每条原始观测（无论提交与否）
     用记录自带 radial yaw 反推中心候选（xy）与板高（z，中位板高=中心高），
     只收近正面记录（std<0.06，std 含系统偏差量级可直接当信任度），
     逐轴窗口**中位数**出锚点，predict 里缓慢写入状态中心；
  2. **静态共享**：候选窗与过零间隔设为 static——管线在翻转风暴中会
     整轨重建（实测 60s 内 38 次 re-init），实例级窗口随之清零导致
     坏锚点反复抽签；outpost 静止且全场唯一，共享安全；
  3. **过零计时测速**：板方位穿越相机方位的时刻记录 yaw 偏差恰为零，
     用相邻过零间隔 (2π/3)/Δt 得 |vyaw|（实测 2.513 分毫不差），符号取
     差分 EMA 符号；predict 里以此牵引 yaw_rate，并把 yaw_acc 向 0 衰减；
  4. 见证/重置输入改为**帧内最正面记录**（pick_best_obs）；见证喂入加
     正面门（std<0.06）+ 板切换双门（|Δ|<0.35 且 dt<0.2s）；
  5. 见证可信时（streak≥5 且相干度>0.85）**K 的 yaw_rate/yaw_acc 行
     置零**——速率只来自见证，相位仍由 Kalman 更新锁定；
  6. 移除半径在线学习（与锚点构成正反馈：中心偏→半径读大→中心更偏）。
- **验证**（armor_pose，全新仿真会话，65s×3）：命中率
  **40.3% / 36.4% / 34.5%**，DPS **111 / 146 / 138**；中心误差 p90
  ≤0.029m，vyaw 误差 p50≈0。两旋向均验证过。
- 剩余问题：tracking 帧占比仍只有 ~20%（大量 state=1 帧仍在射击且命中
  贡献不小）；good-basin 外偶发 episode 未完全根除；vision 模式下新链路
  只冒烟未调参。

### 2026-07-26：WSL interop 修复与 debug 窗口缩放（已完成）

- 问题 1：`run_gestalt_stdio.sh` 报 `Exec format error: /mnt/c/Windows/py.exe`。
  根因是 `/proc/sys/fs/binfmt_misc/WSLInterop` 注册丢失（systemd-binfmt 被
  WSL 跳过且 binfmt.d 无注册项）。修复：手动注册 + 写入
  `/etc/binfmt.d/WSLInterop.conf` 持久化；gestalt 链路冒烟全程跑通。
- 问题 2：`--debug` 窗口在高 DPI 屏幕上变得很小。根因是 WSLg/XWayland 对
  X11 应用按 96 DPI 呈现（不随 Windows 缩放），OpenCV `resizeWindow` 在该
  后端不可靠。修复（`e6c3fa2`）：`bringup_sim` 直接按
  `HFUT_DEBUG_WINDOW_SCALE`（默认 1.0）缩放显示图像，不影响算法与截图。
- 注意：同日排查矩阵无帧时发现 `configs/gimbal_pipeline.yaml` 被手动改为
  `bridge_path: gestalt`，导致矩阵算法端等 gestalt 相机流、全程
  "no camera frame" 且无显式报错；已恢复 webots 默认值。**该文件是共享
  状态，手动改模式后请改回，否则矩阵静默失效**（已记入 §8 环境坑）。

### 2026-07-30：outpost 跟踪链修复（armor_pose 模式，已完成）

- 任务：让 outpost 靶在 armor_pose 模式下稳定跟踪并击打。
- **总根因**：`OutpostInEKFBackend::initialize_state` 按视觉 PnP 法线约定
  把 obs.yaw 减 π 再反推中心，而 armor_pose 直发记录（pipeline.cpp:387）
  给的是真 radial yaw——初始中心被镜像 2·r=0.55m，下游全部异常由此放大。
- 修复链：①初始化去掉 -π；②**静态锚点**（中心冻结于 reset 锚点，平移
  状态钉零、K 对应行置零）消除 (center,yaw,panel) 流形歧义；③观测噪声
  用直发记录逐板协方差+配置地板并按俯仰投影 xy/z；④装甲方位见证 +
  镜像重锚；⑤门限链放宽（模式入门/出门/提交门/warmup/phase_audit/
  后验中心跳变限幅）；⑥半径与 z_offset 在线 EMA 学习并钳 ±5%（用户
  要求先验可适应）；靶高修正 1.818→1.618（z_offset_0=+0.102）。
- 验证（全新仿真 65s）：TRACKING 1304/1697、LOST 14；中心误差 0.030m；
  vyaw 2.634（真值 2.513）；板位误差 0.038m；**628 发 213 中、命中率
  34.9%、DPS 62.7**（修复前 ~0.3%）。详见 VALIDATION.md 2026-07-30 条目。
- 注意：vision 路径的 outpost（gestalt）未在本次范围；静态锚点假设
  outpost 静止，若未来 outpost 可动需改为慢速锚点跟随。

### 2026-07-30：前哨战（outpost）靶标与启动脚本（仿真仓，已完成）

- 任务：制作 3 板高低错落前哨战靶标 + 配套启动脚本（先做到能旋转、相机
  能看到）。
- 仿真改动（仿真仓，未提交）：新增 `worlds/outpost_world.wbt.template`
  （3 板互成 120°、半径 0.275m、高度 1.414/1.516/1.818m、斜向下 15°、
  距射手 5m、根节点在圆心 1.516m）与 `run_outpost_target_test.sh`
  （0.8π rad/s 匀速、旋向每次运行随机、armor_pose 以 outpost 编号发布）；
  `ros_free_score_system` 改为跳过缺失装甲位（支持 3 板，已重编译）；
  README 补充靶标说明。
- 主仓改动：`tracker.yaml` outpost z 语义对齐新靶（z_offset_0/1/2 =
  +0.302/0/-0.102，相对根节点 1.516m；半径 0.275m 本已一致）。
- 验证：truth 目标位 (5,0,1.516)、转速 -2.513 rad/s（随机方向生效）、
  相机帧可见 1-2 块板（样式/数字与地面靶一致）、armor_pose 记录编号
  outpost、计分系统 3 板无崩溃；配置 env 冲突（target_robot.env 地面靶
  默认值覆盖）已修复为 sourcing 前备份/后强制。
- 算法侧注意：① `selector.blocked_robot_ids` 当前为 ["out_post","negtive"]
  （拼写与 "outpost" 不符，outpost 实际未被屏蔽——是否为有意解除请用户
  确认）；② armor_pose 闭环下 outpost 跟踪器能建轨但 vyaw 估计为 0、
  中心误差中位 0.54m、命中率 ~0.3%，outpost 跟踪链仍属未完成项
  （原屏蔽原因），如需精度应单开任务调试。

### 2026-07-26：配置文件清理与死配置删除（已完成）

- 任务：完善 configs 下 YAML：删除永不调节的死配置、补充可调参数与中文注释。
- 依据：全仓配置项消费盘点（pipeline_params declare/get + 算法实际消费）。
- tracker.yaml（794→约 400 行）：删除 entropy/binder/norm4_v2（含嵌套错误
  导致的 `binder.norm4_v2.*` 死段）/external_targets/panel_mismatch/
  debug_2d_viz/temp_lost_thres/max_match_*/spin 非 delta 键/ukf 遗留观测噪声
  键/outpost v2 全部遗留键/mode_routing/single_plate_bridge/fallback/
  phase_memory 等零消费段；ukf_v1 段补齐此前被错写在死段而从未生效的
  vertical_dynamics_scale 等 7 键（按代码实际默认显式列出）；休眠后端
  ukf_v2/adaptive 保留最小集。
- controller.yaml：删除 logging.*、顶层遗留 solver/state_machine/mpc 兼容段
  及全部 shadowed 兼容键（solver.*delay、state_machine.prediction_delay、
  fire.trigger_to_muzzle_s/flight_time_iters、mpc.*delay*、control_rate、
  bullet_speed——后者恒被 simulation.yaml 覆盖）；新增
  fire.velocity_low_pass（开火速度低通，此前仅有代码默认）。
- detector.yaml：删除 detector.target_frame、runtime.publish_empty/
  copy_policy/profile、pose.sliding.*（无 sliding refiner 实现）。
- gimbal_pipeline.yaml：删除 visualization_frame、debug_mode、
  backend_config.enable_shadow_mode/shadow_convergence_frames。
- 验证：4 文件 YAML 解析通过；ctest 13/13；stationary_L2（TRK 100%、
  ce 5.1c）与 moving_L2（TRK 98%、ce 5.6c/pe 6.5c）冒烟与清理前基线一致。
- 备注：同日发现 gimbal_pipeline.yaml 被手动改为 bridge_path=gestalt 导致
  矩阵静默无帧（gestalt 端点等不到相机流）；已恢复 webots 默认值。修改
  该文件默认值前请先确认当前主用路径。

### 2026-07-25：README 与 Gestalt 文档漂移修正（已完成）

- 任务：更新 hfut_auto_aim 文档与 README。
- README：默认输入改述为 vision；功能范围补 InEKF 默认后端（ukf_v1 可切回）、
  NN+灯条混合角点精化、旋转见证与自适应过程噪声、观测宽容度三项机制、
  panel_switch_hysteresis（默认关）、IPPE 歧义度上报；新增 ctest 13 与难度
  矩阵用法（16 格、INPUT_MODE/STRATEGY 环境变量）；armor_pose 记录更正为
  协议 v3 的 128 B；3D 调试器标注"无法正常工作，待修理"。
- Gestalt 三份文档同步实际行为：stdio 链路默认**允许开火**（
  README/TUTORIAL §6/§8、BRIDGE；安全默认值是否改回禁火仍列为 §6 第 1 条
  待用户决定）；idle scan 更正为 0.5s 等待、yaw 120°/s、pitch 30°/s。
- 验证：仅文档改动，未触碰代码；参数值均与当前源码/配置核对。

### 2026-07-25：vision 角点精化（sp_vision 式 NN+传统灯条混合）（已完成）

- 任务：参考 sp_vision_25 图像处理提升 vision 识别准确度（含对下游的
  影响），跨环境泛化不过拟合。
- 改动：启用仓库已有的 RoiPcaCornerRefiner（原死代码）——每个 NN 检测
  在局部 ROI 内重跑传统灯条拟合（OTSU 自适应阈值 + minAreaRect + 几何
  校验），通过才替换关键点，失败保留 NN 关键点（极端环境零行为变化）；
  灯条内陷补偿：按 2D track 维护 NN板幅/精化板幅 比值 EMA 缩放精化
  角点（精化保形状稳定、NN 保物理板幅，无环境常数）。config_loader 补
  corner_refine 加载；detector.yaml 新增 corner_refine 段。
- 实片效果：IPPE 歧义度中位 0.251→0.000（孪生解闪烁消除）、跟踪状态
  切换 23→3、中心跨度 y 0.26→0.16m。
- 全矩阵（16 格×60s，predicted）：pe_med 均值 13.0c vs 基线 15.1c
  （-14%）、|werr| 0.69 vs 0.92、TRK 96% vs 94%、ce 7.6c vs 7.7c 持平。
  残余：stationary_L2/L3 ce +2.4c 左右（webots 内陷辉光条渲染特性，
  非真实板预期行为，详见 VALIDATION.md）。
- 验证：ctest 13/13；video_test、子矩阵 A/B、全矩阵各一轮。

### 2026-07-25：真值误差口径调参战役（已完成）

- 任务：以真值误差为主指标（ce/pe、ω 误差、更新及时性）调参，覆盖全部
  4 场景×L1-L4；vision 看视觉影响链路，armor_pose 看纯估计/预测链路；
  不为迎合 webots 而调参。
- 基线（ca 5.0）：vision ce 均值 8.0c、pe 15.2c（pe/ce≈1.9）、|werr| 1.67；
  armor_pose ω 全格完美（werr<0.1）——ω 锁 0 为视觉侧（orbit 实测 FOV
  边缘裁切板），pe 滞后为两模式共有短板，预测外推已是 CA。
- 实验：E1 ca_acc 5→8（pe 总和 -5% 无代价，**保留**）；E2 输出平滑提速
  （中性，回退）；E3 预测加速度限幅 8→12（不触限，回退）；E4 vision
  噪声 scale 1.5→1.2（orbit_L2 TRK 97→86% 净负，回退）。
- 定稿验证（vision 16 格，ca 8.0）：ce 7.7c vs 基线 8.0c、pe 15.1 vs
  15.2c、|werr| 0.92 vs 1.67、TRK 94% vs 95%——方差内略优，无回归。
- 结论：当前配置在真值误差口径下已达局部最优；剩余差距（vision ce 2×、
  orbit ω 锁、random_L4）均为视觉/搜视场景层问题，非滤波参数可解。
- 主仓改动：`tracker.yaml` ca_acc 8.0、`tracked_robot_usage.cpp` 注释修正、
  本文档与 VALIDATION.md 记录。

### 2026-07-25：随机走位靶小陀螺增强（仿真仓，未提交）

- 任务：随机移动靶加入更多小陀螺倾向，小陀螺持续时间最少 3 秒。
- 仿真改动：`target_spinner` 小陀螺进入概率 25%→45%（
  `WEBOTS_TARGET_RANDOM_SPIN_PROBABILITY`）、满速保持 3-6s
  （`SPIN_HOLD_MIN/MAX_S`，满速后才计时）、`SPIN_PAUSE_TRANSLATION`
  默认 false（边转边移，用户确认；置 true 则原地旋转）；修复
  `uniform(lo,hi)` 重复加 lo 的存量 bug 与满速计时/退出互锁震荡 bug；
  `difficulty_levels.env` 与 `run_random_spin_target_test.sh` 默认值同步；
  控制器二进制已重编译。
- 验证：L1-L4 真值分析小陀螺段 3.4-5.9s 全 ≥3s、旋转期平移 |v|=0、时间
  占比 23-36%；矩阵 random_walk 子集 TRK 99/94/98/78%。边转边移复测（L3）：4 段 3.4-5.8s 全 ≥3s，旋转期平移 |v| 中位 1.36 m/s。
- **注意：random_walk 场景语义已变，此前的 random_walk 矩阵数字不可与
  之后直接比较**（VALIDATION.md 有记录）。仿真仓改动仍未提交，连同 §4
  既有未提交项一起留待用户检查。
- 主仓改动：仅 `docs/VALIDATION.md`/`docs/HANDOVER.md` 记录。

### 2026-07-25：观测宽容度（歪预测器不再拒准观测）（已完成）

- 任务：提高对观测的宽容度——预测器发散时准确观测不应被持续拒止；
  明显异常观测仍舍去。
- 改动（本提交）：①InEKF 后验中心跳变超 0.30m 由整帧拒止改为限幅提交
  （`_cjc` 诊断标记）；②TRACKING 中连续 2 帧严格门失败但 init_relax
  放宽门通过即采信 top1 观测（单帧 glitch 仍拒）；③
  `reject_reset_streak_frames: 12→6`。
- 验证：mpc 1740 vs 1712（+1.6%）、predicted 1790 vs 1840（-2.7%），均在
  ±8% 方差内；弱格拒止减半（moving_L4 33→11、random_L2 53→27）；
  mpc moving_L4 ω 7.69（历史最佳）、moving_L4 TRK 86→94%。
- 结论：机制目标达成（歪预测器自愈显著加快）且总量无回归，定稿。

### 2026-07-25：任务 1（vision 抖动虚假平移）与任务 2（4 板切 InEKF）（已完成）

- 任务 1（提交 `a8a83e8`）：实片复现静止目标 |v| 中位 0.47m/s、中心跨度
  0.39m。离线复算确认 IPPE 孪生解逐帧闪烁（141 跳变帧全为近简并分支
  切换）与板位置重尾噪声（p50 3.6cm/p99 20cm）。实测排除 5 种干预
  （分支滞回自锁、R_yaw 膨胀破坏锚定、平移见证自适应 Q 门控抖动、
  delta_rate 提高、singer_sigma 降低过拟合），webots 验证环境基线良好
  （中心误差中位 4.1cm）。保留：IPPE 歧义度等效 σ 上报链路（纯元数据）、
  `panel_switch_hysteresis` 机制（默认关）。v15 矩阵 16 格确认无回归。
- 任务 2（本提交）：`backend_type: "inekf"` 定稿。补齐旋转见证+自适应
  旋转 Q、V1 风观测噪声（共享头文件）、±π 折叠、有符号 dza、后验对齐；
  关键调参 `inekf_runtime.ca_process_noise_acc: 5.0`（0.1 塌陷速度先验致
  高难度格拒止率 11% vs 基线 4%）。矩阵（16 格×60s）：mpc 1712 vs 1421
  （+20%）、predicted 1840 vs 1470（+25%）；ω 高档全解锁；random_L4 与
  stationary_L3 格与基线互有胜负（方差/已知边界）。vertical_dynamics_scale
  0.02 与 dual_height_evidence 0.15 经 A/B 判定有害/存疑，默认关闭。
  video_test 实片 InEKF 略优于 ukf_v1（|v| 中位 0.39 vs 0.47）。
- 注意：难度矩阵已扩为 16 格（新增 orbit_spin 场景）；矩阵脚本默认策略为
  mpc（`HFUT_MATRIX_STRATEGY=predicted` 覆盖）；矩阵 run 间方差总量 ±8%、
  单格 ±15%，单格 A/B 需谨慎下结论。
- 验证：build 通过、ctest 13/13、四轮全矩阵（inekf mpc ×2 + ukf/inekf
  predicted 各一）、多轮子矩阵；所有临时仿真进程已退出。
- 文档：详细数据与逐项排除记录见 `docs/VALIDATION.md` 末尾两个
  2026-07-25 条目。

### 2026-07-20：非水平目标夹具与控制模式配置（进行中）

- 任务：为任务 6 增加两类可分离的非水平目标仿真参数，并为任务 4 的 Webots
  云台执行器补齐 MPC 状态模式配置入口。
- 仿真改动：`WEBOTS_ARMOR_TARGET_ROLL/PITCH` 先转为轴角，包裹整车 body/布局
  子树；`WEBOTS_ARMOR_LAYOUT_ROTATION` 仍只作用于装甲板布局面。目标控制器继续
  只更新 Robot yaw，因此整车斜面姿态和装甲板安装倾角不会被覆盖。
- 控制配置：`config/camera_robot.env` 增加 `WEBOTS_GIMBAL_COMMAND_MODE` 及
  `WEBOTS_GIMBAL_MPC_*` 默认项；设为 `mpc_state` 时桥接器消费 MPC 的速度/加速度
  前馈，默认 `position` 保持 predicted 基线行为。
- 验证：`WEBOTS_ARMOR_TARGET_ROLL=10deg`、`PITCH=-5deg` 的
  `run_stationary_spin_target_test.sh --render-only` 成功，生成 world 中出现独立
  body attitude 轴角；尚未完成整段闭环姿态误差统计。
- 剩余：跑两类姿态的 armor_pose 闭环，确认 tracker 的 plate quaternion、中心姿态
  与控制目标一致；随后做 blocked target 端到端和可控响应延迟 A/B。

### 2026-07-20：任务 2-6 现场回归补充（进行中）

- 任务 2 高视角探针：固定相机 yaw=0、目标 yaw=1.0、随机噪声全关时，
  `view_angle=1.073rad` 板测距比 1.082、径向 yaw 偏差 -0.225rad；
  `view_angle=0.626rad` 板测距比 1.045、yaw 偏差 +0.070rad。系统性偏差方向
  与视角一致，详见 `VALIDATION.md`。
- 任务 3 端到端：`WEBOTS_DIRECT_ARMOR_NUMBER=outpost` 的 221 帧全部被入口
  屏蔽，tracked=0、正常命令=0、fire=0。注意主仓 `run.sh` 必须显式传入与
  Webots 相同的 `WEBOTS_ROS_FREE_BRIDGE_DIR`，否则会读到旧序号的默认记录。
- 任务 4：`run_difficulty_matrix.sh` 增加 `HFUT_MATRIX_INPUT_MODE`，可用
  `armor_pose` 做专项矩阵；Webots 配置增加 `WEBOTS_GIMBAL_COMMAND_MODE` 和
  `WEBOTS_GIMBAL_MPC_*`，`mpc_state` 直接消费速度/加速度前馈；桥接器现会在
  no-valid 时清空 stale active 状态、按 50ms horizon/lead 对齐状态，并丢弃与
  位置误差反向的速度前馈。临时 `strategy=mpc + mpc_state` 最新 514 帧中
  TRACKING/TEMP_LOST=333/41、direct 空帧 158、命中 5/6，较初版明显改善但仍
  低于 predicted；默认仍保持 predicted，MPC 留待单独调参。
- 任务 5：bringup 诊断 `delay` 对象新增 `control_latency_s`、控制步数、延迟
  模型和双重补偿标志。80ms yaw/50ms pitch 注入下，补偿档记录
  `control_latency_s=0.08`、`double_compensation_risk=false`，总预测时间约
  0.145→0.223s；补偿已真正进入 predicted 控制目标，严格同轨迹 DPS A/B 尚待做。
- 任务 6：整车 roll/pitch 与布局旋转的真值四元数法向误差分别 `6.3e-5`、
  `2.3e-4`；两类 276 帧闭环均无 LOST，TRACKING 各 262 帧。姿态输出约
  `(-10deg,+5deg)` 与 `(-10deg,0deg)`，坐标符号符合现有 estimator 约定。
- 验证：主仓 `./scripts/build.sh` 与 `ctest` 7/7 通过；仿真桥重编译通过，
  `git diff --check` 通过。完整 armor_pose 12 格矩阵仍未完成。
- 后续发现并修复任务 2 对任务 1 的闭环回归：系统偏差首轮使 stationary
  L1/L2 每次换板出现 25/52 次 `all_gate_fail`。V1 门控与更新现统一使用
  direct/PnP 协方差并保留 yaw 噪声地板；发布器逐板协方差同时覆盖随机噪声和
  实际注入的系统 yaw/测距偏差幅值。复测 stationary L1/L2 分别 624/624、
  628/628 更新全 commit，TEMP_LOST=0，命中率 98.1%/90.1%；random L2
  591/591 commit、TEMP_LOST=0、命中率 94.1%。协议 v3 不变。
- MPC 执行器专项又补了三层保护：no-valid 清空 stale active 状态，horizon/lead
  默认 50ms 对齐 `dt=0.05s`，位置误差反向时丢弃速度前馈，并把 MPC 专用速度
  前馈默认限幅为 yaw/pitch=2/3rad/s。复测 514 帧中 TRACKING/TEMP_LOST=333/41、
  direct 空帧 158、命中 5/6；相比初版明显改善，但仍低于 predicted，因此不切默认。
- 主仓 `scripts/run.sh` / `bringup_sim` 新增 `--strategy=mpc` 单进程覆盖，已用
  无输入启动审计确认 Pipeline ready 显示 `control_strategy=mpc`；默认 YAML
  仍为 predicted。

### 2026-07-20：armor_pose 主链专项调试路线（进行中）

- 总目标：先在 `armor_pose` 模式隔离图像检测/PnP，专项调试
  状态估计 → 跟踪 → 预测 → 控制；在云台运动能力覆盖目标角运动的前提下，
  追求所有标靶尽可能高的 DPS、准确且稳定的连续跟踪。
- 执行顺序与实时状态：
  1. **进行中**：复现并消除连续 armor pose 输入下，快速变向、靠近/远离时
     仍发生的意外 tracker 重置；消除随机移动不旋转靶停转后仍判旋转的问题；
     修复后做 armor_pose 全场景难度矩阵回归。
  2. **待办**：任务 1 达标后优化 Webots armor pose 发布器，模拟大朝向角时
     PnP 的 yaw 与测距系统性失真，而不只添加独立高斯噪声。
  3. **待办**：修复 `blocked_robot_ids` 设置后仍会锁定屏蔽对象的问题，并覆盖
     detector/tracker/selector/control 回退路径测试。
  4. **待办**：任务 2 完成后，联合配置 Webots 与自瞄侧 MPC/轨迹优化并回归。
  5. **待办**：核查预测、处理、控制、触发到出膛的延迟补偿是否真正进入控制量，
     为 Gestalt/实车不可避免的响应延迟建立可测补偿方法。
  6. **待办**：测试两类非水平目标：(a) 布局面与底盘平行、整车在斜面运动；
     (b) 车体水平、装甲板布局面相对底盘倾斜。
  7. **暂停**：Gestalt 仅用于最终效果测试，当前不能提供足够中间数据，暂不用于
     参数调试或根因定位。
- 参考约束：可借鉴 `/root/reference/sp_vision_25`，但每项机制必须用本仓诊断数据
  和可复现 A/B 验证，不能直接照搬参数。
- 当前进展（任务 1 基线，尚未改算法参数/代码）：
  - 纵深往返、不旋转、2m/s、8m/s²、60s：1733 帧中 TRACKING 1730，显式
    reset/删除为 0；当前中心误差 median 6.9cm / p90 19.0cm，预测中心误差
    median 13.6cm / p90 41.7cm，证明即使连续直接位姿也有明显状态滞后。
    本格复用了旧 bridge 目录，score 累积值不可引用。
  - 第一轮“随机不旋转”因把未使用的 spin/follow/wander 加速度/速率也设为 0，
    触发 `target_spinner` 参数校验并退出；该组静止靶 100% 命中数据已判无效，
    不进入结论。后续每格改用独立 bridge 目录并确认 target controller 存活。
  - 有效随机平移固定 yaw（seed42、2m/s、8m/s²、90s）：2437 帧中 TRACKING
    2434，无 reset/删除；命中率 62.4%，eDPS≈225（1014 hits×20/90s）；
    当前中心误差 median 6.2cm / p90 10.2cm，预测中心误差 median 9.8cm /
    p90 22.5cm。真值 yaw 基本固定时估计 `|yaw_rate|>0.1` 有 371/2437 帧，
    最大 0.479rad/s，停转残余角速度问题可复现但尚未达到错误小陀螺阈值。
  - random L4 armor_pose（seed42、2m/s、8m/s²、spin 8、120s）：2997 帧中
    TRACKING 2173、TEMP_LOST 129、无控制 695；命中率 60.2%、score DPS 142
    （含启动空闲期）。本格无 `reject_streak_reset`，仅 20 次融合拒止；实际有
    254 帧 `direct_armors` 为空，armor-pose 发布端的 FOV/朝向裁剪形成多段超过
    `lost_thres=20` 的空窗，触发 LOST 删除/重建。故此格重置属于输入空窗，
    不能证明“连续位姿仍重置”。
  - 下一步：跑纵深往返 + 8rad/s 自旋的持续可见压力格，捕获 direct_armors
    非空时的 `reject_streak_reset` 及其 NIS/posterior/reconstruction 原因；
    随后扩充诊断字段并做针对性修复。
  - 诊断增强已完成：逐帧 JSONL 新增 commit、观测数、top1 NIS/位置与 yaw
    chi2、假设名和决策原因；direct armor 新增实际位置/yaw 噪声与 view angle；
    `posterior_sanity` 失败细分到 center/yaw/radius/dza/covariance。build 与 ctest
    7/7 通过。
  - 同场景 45s 诊断复测：6 次 posterior 拒止全部为双板结构半径单帧跳变
    超过 0.05m（最大 0.0765m），没有 center/yaw/dza/covariance 后验失败。
    根因是 `vehicle_tracker.ukf_v1.dual_update` 对高角度噪声板仍使用 1.0 全结构
    增益，导致整帧有效运动观测也被连带丢弃。
  - 下一步 A/B：参考 sp_vision_25 普通四板目标 0.20m 初始半径，将默认结构
    由 0.15/0.20/0 调为更中性的 0.20/0.20/0，并将双板 r/dza 结构增益由
    1.0 降到 0.25；单板结构增益继续为 0。先在相同压力格验证 TEMP_LOST、
    结构收敛和预测误差，再决定是否保留。
  - 上述 A/B 首轮结果：半径跳变基本消失，但 1318 帧中 TRACKING 1242、
    TEMP_LOST 72，劣于短时基线的 TEMP_LOST 8；新拒止几乎全部变为
    `posterior_dza_range`，trial dza 约为 -0.001~-0.010m。结构误差 median
    3.44cm / p90 3.86cm。根因不是新观测失效，而是 V1 在 trial 物理钳制
    之前执行 posterior range 检查：本可投影到 dza=0 的边界解被当作整帧
    非法更新丢弃。
  - 当前修复：保持 `dual_update.structural_gain_r=0.25`，将 trial 的
    r1/r2/dza 物理约束提前到重建误差与 posterior_sanity 之前，同时保留
    `max_r_jump/max_dza_jump`，避免以放宽门限的方式吞掉真正的大幅结构异常。
    下一步仍用同一 depth 往返 + 8rad/s 自旋 45s 压力格复测。
  - 物理投影版复测：1322 帧中初始化 3、TRACKING 1319、TEMP_LOST 0，1319 次
    tracker update 全部 commit；当前中心误差 median 3.83cm / p90 8.18cm，预测
    中心误差 10.98cm / 26.45cm。连续性目标达到，但 dza 全程被钳在 0，r1/r2
    收敛为约 0.175/0.200，说明该修复丢失了相位对应的结构信息，不能定稿。
  - 修正结论：第一块可见板定义的 panel 相位是任意的；状态
    `[r1,r2,dza]` 与 `[r2,r1,-dza]` 相差 90deg 相位但代表完全相同的物理
    布局。全局 `constraints.min_dz=-1` 与 sp_vision_25 的有符号高度差模型也
    佐证 dza 不应强制非负。当前改为 V1 posterior 对称范围 [-0.15,0.15]，
    仍保留单帧 `max_dza_jump=0.03`；trial 在原始后验检查通过后再按该范围
    投影。`diagnose_tracking.py` 的结构误差同步改为在两种等价相位中取最小值。
  - 有符号 dza 版同场景复测通过：1320 帧中初始化 3、TRACKING 1317、
    TEMP_LOST 0；1317 次有效 tracker update 全部 commit，无 posterior/gate
    拒止或重建。结构收敛到等价相位 r1≈0.171、r2≈0.198、dza≈-0.030m；
    当前中心误差 median 3.31cm / p90 7.80cm，未来时刻预测中心误差 9.86cm /
    25.21cm，控制目标误差 19.97cm / 34.83cm。45s 算法窗口内 142 hits / 174
    shots，命中率 81.6%，按算法数据跨度约 42.1s 计算 eDPS≈67（score 自带
    elapsed=59.84s 含算法启动前仿真空跑，不能直接作为 eDPS 分母）。
  - 阶段结论：连续 direct armor 输入下由结构后验拒止引发的 TEMP_LOST 已在该
    压力格消除；`default_r1/r2=0.20/0.20`、dual r/dza gain=0.25 与有符号 dza
    范围暂作为候选保留。下一步用不同启动相位复验，并回到随机不旋转场景处理
    停转残余 yaw-rate 与快速变向预测误差。
  - `tools/diagnose_tracking.py` 已扩充：自动汇总 direct armor 空帧/最长空窗、
    tracker commit 与拒止分类、各状态连续段、估计 yaw-rate 分布、真值静止时
    残余 yaw-rate 及相对真值误差；结构指标支持 90deg 等价相位。旧的随机平移
    固定 yaw 90s 基线重算后，2434 个静止真值样本中 `|yaw_rate|>0.1` 仍有
    371 帧，p90 0.125、最大 0.479rad/s；direct 空帧 0、状态全程连续，确认这是
    旋转状态估计残余而非输入或 tracker 生命周期问题。
  - 当前候选在随机平移固定 yaw（seed42、2m/s、8m/s²、60s）复测：1735 帧中
    初始化 3、TRACKING 1732、TEMP_LOST 0，1732 次更新全部 commit；当前中心
    误差 median 1.66cm / p90 4.49cm，未来预测中心 2.81cm / 15.44cm；887/1186
    命中、命中率 77.4%，按算法跨度约 55.4s 得 eDPS≈320。残余 yaw-rate p90
    0.103、最大 0.326rad/s，`|w|>0.1` 182 帧，超过 0.1 的段最长 5 帧，属于
    短促噪声尖峰而非持续旋转状态。
  - yaw-rate 控制输出 A/B：滤波器内部仍保留慢速状态供 hypothesis/association
    使用，仅将 smoother 的发布死区由 0.05 调到 0.5rad/s，与选板“低速段”边界
    对齐。0.5rad/s 在 0.2m 半径、约 0.15s 飞行前瞻内切向位移约 1.5cm，低于
    小装甲宽度容差；目标是完全隔离静止 PnP yaw 尖峰，又不影响真正的小陀螺。
    下一步复跑固定 yaw，并对自旋启动/停止场景验证进入与退出延迟。
  - 发布死区固定 yaw A/B 通过：45s 中初始化 3、TRACKING 1297、TEMP_LOST 0，
    1297 次更新全部 commit；所有 1297 个控制帧发布 yaw-rate 均严格为 0。
    当前中心误差 1.73cm / 5.45cm，未来中心 4.29cm / 17.50cm，控制目标
    4.37cm / 14.40cm；662 hits、算法跨度约 41.5s，eDPS≈319，与修改前约 320
    持平。该 A/B 证明静止误判可在控制输出层消除且未牺牲同场景有效 DPS。
  - 随机启停 3rad/s + 随机平移 75s：2165 帧中 TRACKING 2154、TEMP_LOST 8，
    8 帧全部对应 direct armor 短空窗（3 段、最长 4 帧），有观测的 2162 次更新
    全部 commit。4 段真值自旋的发布 yaw-rate 启动延迟 96/96/128/192ms；退出
    滞后约 -96/64/96/608ms。真值静止 1655 帧中 1645 帧发布 0，但仍有 10 帧
    残余（最大 2.15rad/s），且一次在停转约 7s 后出现 4 帧假旋转，说明 0.5
    输出死区显著改善但未完全根治。下一步用 `HFUT_DEBUG_OMEGA_EMA` 对同 seed
    复现，区分 witness 衰减、板号切换与 UKF 状态残留。
  - `HFUT_DEBUG_OMEGA_EMA` 复现确认：49.54s 的 `angle_witness` 在真实自旋已于
    42.3s 停止后仍打印 `ema=-2.67`，原因是板号跳变时旧 EMA 从未清空；随后
    对称陷阱修正把状态 yaw-rate 写大，形成约 4 帧假旋转。已修复为：无效板间
    角差清空 witness，并要求连续 5 个有效同板样本才允许对称陷阱修正；等待
    build/ctest 及同 seed 回归验证。
  - witness 修复 + 同 seed 60s 回归通过：1713 次有观测更新全部 commit；真值
    静止 1248 帧中仅 4 帧发布残余旋转，全部集中在一次真实停转后的 96ms 内，
    不再出现停转数秒后的假旋转。4 段真实自旋启动延迟为 0/128/96/192ms，
    退出误差约 -128/0/128/-96ms。TRACKING 1705、TEMP_LOST 8；8 个 TEMP_LOST
    与 direct armor 25 帧短空窗重合，更新拒止为 0，不属于连续位姿滤波重置。
    当前/预测中心误差 median 2.35/6.12cm，p90 6.11/19.37cm；控制目标误差
    6.11/15.03cm。阶段 1 的“停转后错误判旋转”根因已修复，剩余是输入 FOV
    空窗和单次停转边界的约 0.1s 响应延迟。
  - witness 后的 8rad/s 纵深回归：yaw-rate 仍准确（median 8.002、p90 8.112，
    真值误差 p90 0.176rad/s），但在目标到 1m 近距换向时出现另一条失败链：
    22.080s 首次 gate fail，22.112s 宽松重捕获 trial 的 center jump=0.269m，
    仅因超过固定 0.25m 被拒；随后 posterior/reconstruction 连续拒止，最终
    形成 54 帧 FOV 空窗并删除重建 tracker。该问题与 witness 无关，属于用户所述
    “靠近/远离、快速变向”重置。
  - 针对性 A/B：仅将 V1 `posterior_sanity.max_center_jump` 从 0.25 调为 0.30m，
    与已有 `max_reconstruction_pos_error=0.30m` 对齐；NIS、yaw、r/dza 及重建门
    保持不变。下一轮固定在仿真约 8s 启动，复现相同滤波历史，验证 0.269m
    修正能否阻断后续发散，而不是用全局放宽掩盖错误 hypothesis。
  - 0.30m 候选、固定仿真 8.032s 启动的同轨迹 45s 回归通过：1324 帧中初始化
    3、TRACKING 1321、TEMP_LOST 0、direct 空帧 0，1321/1321 更新 commit；
    22.0-22.5s 的 1m 近距换向段每帧连续提交。yaw-rate median 7.999、真值误差
    p90 0.163rad/s；当前中心误差 3.34/8.48cm，未来中心 9.36/25.11cm；
    150/171 命中（87.7%），算法跨度约 42.3s，eDPS≈71。由于 Webots 噪声
    未固定随机序列，该结果证明候选无回归并覆盖一次同轨迹换向，但不能作为
    严格逐样本 A/B；需在后续多格矩阵继续观察 center posterior 拒止计数。
  - random L4（半径 2m、2m/s、8m/s²、随机 8rad/s、90s）边界复测：2622 帧中
    有效输入 1977、commit 1940、all-gate-fail 37；direct 空帧 732（28%，最长
    136 帧），TRACKING 1843、TEMP_LOST 134，命中率 64.2%。该格目标自旋
    8rad/s（约 458deg/s）已超过当前云台角速度能力，空窗/状态段不能归因于
    连续 armor_pose 滤波重置；它保留为超能力边界与搜视/MPC 配置的后续测试。
  - 本轮验证收束：`./scripts/build.sh` 成功，`ctest` 7/7 通过，`git diff --check`
    通过；所有 Webots/target_spinner/bringup_sim 临时进程已确认退出。当前任务 1
    已完成针对性根因修复（双板结构后验拒止、相位导致的有符号 dza、停转旧
    witness、近距重捕获窄门、静止 yaw-rate 发布尖峰），但“所有难度×所有标靶”
    的完整矩阵尚未跑完，故整体仍标记为进行中。任务 2（PnP 大角度系统失真）、
    任务 3（屏蔽回归已由 `target_selector_blocked` 单测覆盖但需端到端确认）、
    任务 4-6 尚未开始；任务 7 Gestalt 继续暂停。
  - 任务 2 已开始并完成发布器首版：仿真 `ros_free_camera_bridge` 的
    `armor_pose` 不再只有独立高斯噪声，新增随 `sin(view_angle)^exponent` 增长的
    PnP 风格系统失真：radial yaw 向相机方位收缩，位置沿真实相机光线按比例偏移，
    板面四元数同步施加 yaw 偏差。默认参数为 yaw gain=0.25、range ratio=0.10、
    bias exponent=1.5，均可用 `WEBOTS_DIRECT_ARMOR_*_BIAS_*` 环境变量标定。
    仍保留 max-view 裁剪和随机噪声元数据，协议 v3 不变。
  - 任务 2 验证：桥接器重新编译通过；极端探针（随机位置/yaw 噪声=0、range
    ratio=1.0、yaw gain=0）在 view=0.096rad 时实测距离比 1.0299，与公式
    `1+sin(view)^1.5` 一致。真实默认参数下的高视角统计受板间最近匹配和随机
    噪声影响，尚未作为最终标定结论；需要后续固定板 ID/角度夹具做曲线回归。

### 2026-07-20：交接复核与现场状态纠偏

- 任务：重新阅读 HANDOVER 及其相关文档，以现场仓库为准建立后续持续记录基线。
- 已阅读/核对：主仓 README、VALIDATION、UNMIGRATED_FEATURES、三份 Gestalt 文档、
  五份生效配置、`f9f3aa3` 提交内容；仿真仓 README、工作区 diff、难度矩阵脚本、
  bridge 协议 v3 与倾斜布局夹具。
- 现场纠偏：主仓算法基线已推进到 `f9f3aa3`，默认输入现为 `armor_pose`；
  仿真仓除原有 seq/矩阵改动外，还有协议 v3、完整板面姿态和倾斜布局改动，
  仍未提交。
- 文档/安全发现：stdio Gestalt 启动实际默认允许开火，且 idle scan、默认 entity、
  armor_pose record 大小与部分教程不一致；已列为 §6 最高优先级，未擅自改动行为。
- 本轮只修改本文档，未改算法或仿真代码。验证：两仓 Git 状态已核对；主仓 ctest
  在允许本地回环 socket 的环境 7/7 通过。受限沙箱内仅 gestalt_bridge_client
  会因 `socket()` 被拒绝而显示 6/7，不是测试断言回归。
- 约定：后续每项实质工作继续按时间倒序写入本节；详细实验数据同步写
  `docs/VALIDATION.md`，不依赖对话上下文保存结论。

### 2026-07-19：可视化/屏蔽/响应延迟/姿态符号批次（已完成）

- 任务：修复五个典型问题——估计可视化只在 AIM 态出现、屏蔽应在
  detector 生效、单板错位（估计错位.png）、双方运动偏移（误差.png）、
  PREDICT_ONLY 应保持固定时长。
- 改动与验证（详见 VALIDATION.md 末节）：
  1. 估计板全程可见：pipeline 填充 tracked_armor_poses，bringup_sim 在
     非 AIM 且轨迹存活时持续绘制估计整车板。
  2. 屏蔽下沉 detector 级：updateTracking 入口丢弃 blocked_robot_ids
     装甲板，跟踪/选择/姿态不再接触（selector 保留兜底）。
  3. PREDICT_ONLY 快速重置：top1 改用 init_relax 放宽门 + 无门 force
     重置（streak≥lost_thres-4），moving L3 实测 TEMP_LOST 段中位 1 帧。
  4. 姿态估计符号自校准：±mount_pitch 两候选取竖直者，平台无关，
     mount_pitch_deg 回物理值 +15.0，平地 roll +0.01°/pitch −0.27°。
  5. 单板结构学习 0.05 试验：矩阵轻微变差（r1/r2 随噪声游走），已回退
     为 0；单板错位机制保留为未决项。
- 回归（v14 六格）：moving_L1 168（历史最佳）、stat_L1 260，批次净中性
  偏正；单测 7/7。
- 未决（gestalt 专属）：单板错位的结构剖面/标定、gestalt 元数据位姿与
  图像时间戳对齐核查（双方运动偏移）。
- 当前未提交：pipeline.hpp/cpp、bringup_sim.cpp、body_attitude_estimator.hpp、
  vehicle_tracker.cpp、tracker.yaml、VALIDATION.md/HANDOVER.md 改动在工作区。

### 2026-07-19：装甲板三维布局、预测与可视化修正（已完成）

- 任务：按 0/2 与 1/3 对板分组、`r1/r2/dza`、安装平面可倾斜、
  旋转轴可不垂直安装平面、板面 15° 安装角的约束，修正预测和调试可视化。
- 取证修正（与上一条预判不同，以实测为准）：
  - 15° 板面安装角此前**已正确**：overlay 板 height 轴 z=cos15°、width 轴
    水平（HFUT_DEBUG_AXES 打印验证）；~2m 处仅 ~1° 视角差，视觉不明显。
  - `predict()` 本就保留倾斜（完整角速度向量积分初始姿态，非 Z 轴硬编码），
    几何单测已覆盖；真正被丢弃倾斜的是中心系默认 `yaw_plane` 策略。
  - 关键错误在姿态估计器：`attitude.mount_pitch_deg` 原值 +7°，实测 PnP
    数据试算 −15° 才对（chassis_up 偏差 28.5°/20.5°/**3.6°** 三档对比），
    平地读数由 10-13° 修正到 roll +0.3°/pitch +0.2°。（后续批次已改为
    符号自校准 +15° 物理值，见上一条。）
- 改动（`configs/tracker.yaml`）：`mount_pitch_deg` 修正（后经符号自校准
  定为 +15.0 平台无关值）；`apply_to_geometry: false → true`——姿态并入
  center_pose 后 AUTO 投影自动走 full SE(3)，**可视化与预测都跟随真实
  安装面倾斜**，无可信姿态时回退原 yaw_plane。`apps/bringup_sim.cpp` 加
  HFUT_DEBUG_AXES 调试打印。
- 验证：单测 7/7（含 armor_geometry_3d）；平地仿真姿态读数 ≈0° 且 overlay
  板面正常；矩阵六格无回归（stat_L1 263、stat_L3 242 均为历史最佳）。
- 剩余：倾斜靶车端到端仿真需仿真仓加整车 roll/pitch 参数（记录为后续项）；
  RViz marker 路径（visualization.hpp）同为中心 yaw-only，未改（非调试主链路）；
  旋转轴不垂直安装平面的完整模型（绕布局法线而非世界 Z 自旋）为更大改动，
  当前预测为"世界旋转轴上保留安装平面倾斜"。
- 详细数据：`docs/VALIDATION.md` 末节「装甲板三维布局」。
- 当前未提交：上述 tracker.yaml 与 bringup_sim.cpp 改动在工作区。

### 2026-07-19：接手与上下文核对

- 任务：阅读本文档及相关内容，建立后续进展记录约定。
- 已阅读/核对：`docs/VALIDATION.md`、主仓 `README.md`、
  `docs/UNMIGRATED_FEATURES.md`、Gestalt 文档结构、五份生效配置的关键值、
  主仓近期提交与两仓工作区状态、仿真仓难度矩阵说明。
- 纠偏：主仓改动已于 `d2d477f` 提交，当前干净；仿真仓的 bridge、
  矩阵脚本/报告工具和 README 仍未提交。已据此更新 §4。
- 当前基线：默认 `predicted` 策略；主要未决项为 random_walk L4 近距/搜视、
  MPC 调优、高速转速过冲与移动靶速度估计滞后。
- 验证：本次为文档和 Git 状态核对，未改动代码，未重跑 build/ctest/仿真。
