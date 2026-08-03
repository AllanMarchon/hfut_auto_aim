# ROS2 版（hfut_rm_auto_aim_ws）未迁移功能清单

对照日期：2026-07。ROS2 工作空间包含多版迭代，部分功能已被替换但代码未删除。
本清单按"ROS2 内状态"分组，逐项给出用途与迁移必要性判断，供后续决策。

## 判定依据

- 现役主链路 launch 仅两个：`rm_bringup/launch/bringup_pipeline.launch.py`（实车）与
  `bringup_sim.launch.py`（仿真），启动节点一致：`armor_detector_nn` + `ballistic_solver`
  + `gimbal_pipeline` + 相机源 + 串口（实车 `rm_serial_driver_node` / 仿真 `virtual_serial_node`）
  + `robot_state_publisher`。
- 其余 launch（`bringup.launch.py`、`.single/.debug`、`bringup_decoupled`、`bringup_max_entropy_test`）
  引用的节点包（`armor_tracker`、`gimbal_controller`、`max_entropy_tracker`、`robot_pose_estimator`、
  `target_selector`、`trajectory_planner`、`hero_armor_solver`）已在提交 `6805582` 中物理删除，
  功能并入 `gimbal_pipeline`，且已随迁移进入 ros-free。

## 一、确定"未迁移且现役"（上实车的硬性前提）

状态约定：以下两项已列为**待做（planned）**，但因当前处于仿真阶段、无实车需求，
暂缓实施；待进入实车阶段后按下表补齐。

| 功能 | 用途 | 备注 |
| --- | --- | --- |
| `rm_serial_driver`（rm_serial_driver_node + default/infantry_16/infantry_32/sentry 协议） | 下位机串口通信：云台指令下行、反馈上行，实车唯一硬件通道 | **待做（暂缓）**：ros-free 只有仿真文件桥（`io/gimbal/webots_bridge_gimbal.cpp`）。上实车必须补 |
| `ros2_mindvision_camera` / `ros2_hik_camera` | 迈德威视/海康工业相机驱动，实车图像源 | **待做（暂缓）**：ros-free 只有 `io/camera/webots_bridge_camera.cpp`。上实车必须补 |
| 传统 `armor_detector`（灯条检测+数字分类+BA/图优化位姿） | NN 检测失效时的回退路径，现役 launch 保留为显式回退 | **已迁移**：见 `tasks/auto_aim/detector/` 与 README 检测器选择一节 |

## 二、现役但已被功能覆盖（无需迁移）

| 功能 | 用途 | ros-free 对应物 |
| --- | --- | --- |
| `armor_detector_nn` | NN 装甲板检测（ONNX/OpenVINO/TensorRT、关键点解码、NMS、SolvePnP、位姿细化） | 已迁移：`tasks/auto_aim/detector/` |
| `gimbal_pipeline` | 跟踪→选板→预测→控制→开火全链路 | 已迁移：`tasks/auto_aim/pipeline/` |
| `ballistic_solver` | 弹道解算 ROS 服务节点（重力+阻力迭代） | 进程内 `local_trajectory_compensator.cpp` 已覆盖；另有独立研究仓库 `hfut_auto_aim_ballistic_oracle` |
| `video_player` | 视频文件回放节点（测试图像源） | 文件桥模式天然覆盖（仿真端写 `camera_frame.bin`） |
| `rm_utils` | FYT 日志、PnP、弹道、EKF、URL 解析等 | 已按需 vendored：`third_party/rm_utils/` |
| `rm_interfaces` / `rm_robot_description` | 消息定义 / URDF+TF | `third_party/compat/rm_interfaces` + `configs/simulation.yaml` 外参替代 |

## 三、半成品/实验方向（未迁移，按需决策）

| 功能 | 用途 | ROS2 内状态 | 迁移必要性 |
| --- | --- | --- | --- |
| `auto_buff`（buff_detector + buff_tracker + buff_pose_estimator） | 能量机关（打符）链路 | 包根 `COLCON_IGNORE` 不编译；launch 全注释；`enable_auto_buff: false` | 取决于赛季是否打符；迁移成本中等（三节点）。两侧 gimbal_pipeline 均保留了 buff 外部目标消费口 |
| `armor_fusion`（multi_camera_fusion_node） | 多相机装甲融合（DBSCAN 聚类+小 BA） | demo 级，仅测试 launch 引用（2026-03 单次提交） | 低，未上主链路 |
| `blind_detector` | 哨兵补盲相机检测（armor_detector 裁剪副本） | 2026-04 单次提交；无 launch；串口侧订阅代码被注释 | 低，功能未闭环 |
| `kalmanFilters/muit_obj_tracker` | 通用多目标点跟踪框架（匈牙利匹配、SORT） | 实验品，无消费者，仅 test 可执行 | 低，与现役 4 装甲板专用跟踪体系并行探索 |
| `image_raw_recoder` | ROS 话题/bag 录 MP4（多相机同步录制） | 工具性质 | 低，ros-free 无此概念，可用通用录屏替代 |

## 四、已淘汰（明确无需迁移）

| 功能 | 淘汰原因 |
| --- | --- |
| `armor_solver` | 旧版一体式解算节点，功能由 `gimbal_pipeline` 取代，仅剩失效 launch 引用 |
| `kalmanFilters/filters`（+examples） | KF/EKF 运动模型库，唯一消费者是 `armor_solver` 与实验性的 `muit_obj_tracker` |
| `usb_camera_driver` | `COLCON_IGNORE`，无任何 launch 使用 |
| `rm_auto_aim/todo`、`kalmanFilters/todo` | 设计/竞品分析文档（上科大方案对比等），资料性质 |
| `armor_tracker` 等 7 个已删除包 | 已并入 `gimbal_pipeline` 并随迁移进入 ros-free |

## 结论

- 仿真闭环（gestalt / webots）：**无缺失功能**，现役算法链路已完整迁移。
- 上实车：**串口下行链路** 与 **相机驱动** 已列为待做（planned），仿真阶段暂缓，
  进入实车阶段后实施；传统检测回退已随本轮迁移完成。
- 打符/多相机/补盲：均为 ROS2 侧的未完成方向，建议维持现状，待赛季需求明确再议。
