# Gestalt 接入可设置参数表

> 链路总览：`Gestalt 游戏(Windows)` → `tools/gestalt_bridge_windows.py`(Windows 代理)
> → WSL interop stdio 或 TCP → `bringup_sim`(WSL, `io/gestalt/`)
> → 命令回程 `RBExtAim`。本文按"游戏侧 / Windows 代理 / WSL 启动器 / 算法侧"四组整理全部可设置项。
> 文档出处缩写：`ref-readme` = `/root/reference/sp_vision_25_gestalt_system_bridge/readme.md`，
> `cookbook` = `/root/reference/sp_vision_25_gestalt_system_bridge/docs/ai-cookbook-command-guide.md`，
> `tutorial` = `/root/hfut_auto_aim/docs/GESTALT_WSL2_TUTORIAL.md`，
> `bridge-doc` = `/root/hfut_auto_aim/docs/GESTALT_BRIDGE.md`。

---

## 1. 游戏侧（Steam/UE 启动参数 + WS/控制台命令）

### 1.1 游戏进程启动参数（Windows 终端 A，见 tutorial:146-158）

| 参数 | 含义 | 默认值 | 取值建议或约束 | 出处 |
|---|---|---|---|---|
| `-autostart` | 启动后自动进图 | 无 | 必须；Steam 直启流程的前提 | tutorial:147 |
| `-mapid=4` | 地图 id | 无 | 代理的 prep 身份门按 map 4 验证；勿改 | tutorial:148, readme.md:47 |
| `-nettype=0` | 单机/网络模式 | 无 | 保持 0 | tutorial:149 |
| `-hudhidden=1` | 隐藏 HUD | 无 | prep 阶段自动隐 HUD | tutorial:150 |
| `-execdelay=15000` | autostart exec 延迟(ms) | 无 | 保持 | tutorial:151 |
| `-externalvisioncontrol` | 外部视觉控制授权（Shipping opt-in） | 关闭 | **必须是裸参数**，`-externalvisioncontrol=1` 不生效；与 loopback WS 共同构成授权条件，只放开最小命令白名单 | cookbook:20, tutorial:180 |
| `-visionbridge` | 启动即创建共享内存帧流 | 关闭 | 数据面预开关；发布包流程推荐显式传入（也可用白名单 `UEExec r.VisionBridge.Enable 1` 动态开） | cookbook:21 |
| `-wsbind=127.0.0.1` | AttributeMap/控制 WS 绑定回环 | 回环 | 显式写出防配置漂移；**不得暴露到局域网** | cookbook:22-23 |
| `-windowed` / `-ResX=1280` / `-ResY=720` | 窗口与渲染分辨率 | — | 必须与代理 `--frame-width/--frame-height` 一致（1280x720） | tutorial:155-157 |
| （缺省不加）`-exec="SetMatchStatus 1"` | 自动开赛 | 不加 | **绝不能加**：游戏须停在 prep（MatchStatus=0），由代理接管后才开赛 | tutorial:166-172 |

### 1.2 WS 连接与三个核心 JSON-RPC 方法

游戏每实例开一个 `ws://127.0.0.1:<port>/`，端口动态随机（不查固定 9002），
Shipping 从 sidecar `Saved\ai-selftest\<log>.wsport` 读取；消息格式
`{"type":0,"id":<自增>,"method":"...","params":{...}}`（ref-readme:92, tutorial:371-392）。

| 方法 | params | 含义/约束 | 出处 |
|---|---|---|---|
| `attribute.watchAttributeMaps` | `{"attribute_map_ids":[1..256,...],"watch_type":2}` | 订阅属性表推送（先订 1..256，再订推送中发现的新 map id）；位置 10Hz、云台角 30Hz | ref-readme:98 |
| `console.exec` | `{"command":"<TS 控制台命令>"}` | 执行任意下表命令；fire-and-forget；原生 UE 命令须加 `UEExec ` 前缀 | ref-readme:99, cookbook:78 |
| `rgbCamera.applySettings` | `{"camera":{"enabled":1,"fovDegrees":25,"shutterSpeed":120,"iso":600,"armLength":0}}` | 视觉相机参数；**合并语义**：未写的键沿用上次；**非控制台命令**，是独立 JSON-RPC | ref-readme:100, cookbook:79 |

### 1.3 相机参数（`rgbCamera.applySettings` 的 camera 键）★

| 键 | 含义 | 默认/当前值 | 约束 | 出处 |
|---|---|---|---|---|
| `enabled` | 启用视觉相机 | 1 | 必置 1 | ref-readme:100 |
| `fovDegrees` | 水平视场角(°) | **25** | 必须与代理 `--fov` 及帧门一致（±0.1°容差）；WSL 按每帧实际 FOV 算内参 `fx=fy=width/2/tan(FOV/2)`（1280x720@25° ≈ 2886.9，主点 (640,360)，零畸变） | gestalt_bridge_windows.py:1030,853; tutorial:303-305 |
| `shutterSpeed` | 快门速度 | 120 | 参考配置 | gestalt_bridge_windows.py:1031 |
| `iso` | 感光度 | 600 | 参考配置 | gestalt_bridge_windows.py:1032 |
| `armLength` | 相机吊臂长度(cm) | **0** | 哨兵 Tower 相机预设自带 **400cm 追尾吊臂**（第三人称，破坏枪口外参）；`armLength:0` 收回吊臂切枪管第一人称；复活/重建 pawn 后回落默认，必须重新 apply | ref-readme:83,86; gestalt_bridge_windows.py:1033-1034 |

### 1.4 控制台命令（经 `console.exec` 下发）

| 命令 | 参数 | 含义/约束 | 出处 |
|---|---|---|---|
| `Respawn <pid> <entity_id> <team>` | pid / entity / team | 生成/重生车辆；异步，须等新且更高的 map id 且 HP>0（老尸体快照陷阱） | ref-readme:119 |
| `SetAttribute <pid> <attr_id> <value>` | — | 精确写单车属性（勿用 BatchSet 广播误伤同队） | ref-readme:120 |
| `ExtAimClaim <pid> <0\|1>` | — | 认领/交还外部瞄准；claim 原子保存旧 AITargetMode 并设 90；**每秒重发续租，5s 无续租自动恢复**；调用者不得在 claim 前自行写 90 | ref-readme:121, cookbook:172-176 |
| `UEExec RBExtAim <pid> <yaw°> <pitch°> [fire01] [yawVel°/s] [pitchVel°/s]` | 世界系绝对角（UE 左手系，度） | 瞄准+开火请求；开火仍过热量/弹量/CanOperate 原生门；**0.75s fire watchdog**：无新 fire=1 命令自动清扳机 | ref-readme:122, cookbook:172-173 |
| `UEExec RBTakeOver <pid>` / `UEExec RBTakeOver release` | — | 视口切到该车第一人称枪管视角 / 释放 | ref-readme:123 |
| `SetMatchStatus 1` | — | 开赛（配合 prep 停局实现"先接管后开赛"） | ref-readme:128 |
| `UEExec r.VisionBridge.Enable 1` | — | 开共享内存帧桥（SHM 按游戏 PID 命名） | ref-readme:129 |
| `UEExec r.AntiAliasingMethod 1` / `r.MotionBlurQuality 0` / `r.RobotNav.DebugDraw 0` / `t.MaxFPS <n>` | — | 渲染四件套：FXAA / 关运动模糊 / 关导航调试线 / 限帧（代理 `arm()` 内固定下发） | ref-readme:130, gestalt_bridge_windows.py:557-563 |
| `SetAttribute <pid> 10000033 <n>` | allowance | 代理 prep 时写初始 17mm 弹量配额 | gestalt_bridge_windows.py:554-556 |
| `SetAttribute <pid> 50000088 1` | IsAIControlled | prep 时置内置底盘 AI | gestalt_bridge_windows.py:628 |

### 1.5 第一视角归属（身份门）★

| 项 | 值 | 含义/约束 | 出处 |
|---|---|---|---|
| `player_id` | **0** | 被 RBTakeOver/RBExtAim 控制的玩家；`--prepare-match` 目前要求 player-id 0 | gestalt_bridge_windows.py:1004,1071-1072 |
| `entity_id` | **66000005** | 哨兵 HACHISEN（SKM_Sentry 底盘）；其他：步兵 66000002、突击步兵 66000008、英雄 66000001 | ref-readme:119, gestalt_bridge_windows.py:1016 |
| `team_id` | **0（红方）** | TeamId 0=红 1=蓝； prepared HACHISEN 属红方，故算法侧敌方为蓝 | ref-readme:107, simulation.yaml:46-49 |
| `Class` | **1004** | 属性 `60000002`，哨兵类别；身份四重验证之一 | ref-readme:108, gestalt_bridge_windows.py:499 |
| `ConnectionEntityConfigId` | **66000005** | 属性 `10000064`，玩家表指针反查验证 | gestalt_bridge_windows.py:59,459-468 |
| 四重验证 | PlayerID=0 + TeamID=0 + Class=1004 + EntityConfig=66000005 | 同时满足且 HP>0 且 map id 更新才认为 spawn 成功；3 次重试后硬失败 | gestalt_bridge_windows.py:491-502,636-641 |

### 1.6 代理使用的属性 id（`gestalt_bridge_windows.py:54-62`）

| id | 常量 | 含义 |
|---|---|---|
| 1000001 | `A_MAP_PTR` | 战斗 map 指针（自动发现入口） |
| 10000003 | `A_HEALTH` | Health |
| 10000033 | `A_ALLOWANCE_17MM` | 17mm 弹量配额（=0 则 FiringLocked） |
| 10000035 | `A_PLAYER_ID` | PlayerId |
| 10000036 | `A_TEAM_ID` | TeamId（0红 1蓝） |
| 10000064 | `A_CONNECTION_ENTITY_CONFIG_ID` | 玩家表→entity config |
| 60000002 | `A_CLASS` | Class（哨兵 1004） |
| 50000088 | `A_IS_AI` | IsAIControlled |
| 80000005 | `A_MATCH_STATUS` | MatchStatus（-1未知 0prep 1进行 2结束） |

> Shipping 授权矩阵（cookbook:81-104）：外部视觉白名单仅 `RBTakeOver`、`RBExtAim`、
> `r.VisionBridge.Enable`、`r.MotionBlurQuality`、`r.AntiAliasingMethod`、
> `r.RobotNav.DebugDraw`、`t.MaxFPS`；`RBNavLab`/`RBNavGoto` 属自动化白名单
> （需 loopback + `-automationcontrol`），三靶 bench 用，正式外控不可用。

---

## 2. Windows 代理 `tools/gestalt_bridge_windows.py`

### 2.1 argparse 参数（全部，`gestalt_bridge_windows.py:995-1047`）

| 参数 | 含义 | 默认值 | 取值建议或约束 | 出处（行号） |
|---|---|---|---|---|
| `ws_port`（位置参数） | 游戏回环 WS 端口 | `auto`(0) | `auto` 按 Gestalt 进程名筛 listener 并 WS 握手确认；整数须在 [1,65535] | 998-1000, 154-163 |
| `--transport` | WSL 帧/命令传输 | `tcp` | `tcp` / `stdio`；主路径 stdio（WSL interop，不走 IP）；TCP 为兼容回退 | 1001-1003 |
| `--player-id` | 被控玩家 | 0 | `--prepare-match` 要求 0 | 1004-1005, 1071-1072 |
| `--game-pid` | 限定自动发现的监听 PID | 0 | 多开游戏时指定；共享帧与 WS 必须同 PID | 1006-1007 |
| `--endpoint-timeout` | 等待游戏 WS 秒数 | 60.0 | >0；地图加载慢时加大 | 1008-1009, 1067-1068 |
| `--prepare-match` | prep 中生成并验证红方 HACHISEN 再接管 | 关 | 游戏须停 prep（MatchStatus=0）；已开赛（≥1）会被拒 | 1010-1012, 607-613 |
| `--start-match` | 帧契约通过后发 `SetMatchStatus 1` | 关 | **必须搭配 `--prepare-match`** | 1013-1015, 1069-1070 |
| `--entity-id` | prepare-match 生成车辆 entity | **66000005** | 哨兵 HACHISEN；见 §1.5 | 1016-1017 |
| `--team-id` | prepare-match 车辆队伍 | 0（红） | 0红/1蓝 | 1018-1019 |
| `--allowance` | 初始 17mm 弹量配额 | 400 | ≥0 | 1020-1021, 1073-1074 |
| `--listen` | TCP 绑定地址 | `0.0.0.0` | WSL2 NAT 模式必须 0.0.0.0；mirrored 可用 127.0.0.1 | 1022-1023 |
| `--tcp-port` | TCP 监听端口 | 47000 | [1,65535]；防火墙仅允许专用网络 | 1024, 1065-1066 |
| `--allow-client` | WSL IPv4 白名单（逗号分隔） | `auto` | auto = 127.0.0.1 + `wsl hostname -I`；非默认发行版手工指定 | 1025-1027, 861-881 |
| `--frame-width` / `--frame-height` | 帧门期望分辨率 | 1280 / 720 | >0；必须与游戏 `-ResX/-ResY` 一致 | 1028-1029, 1077-1078 |
| `--fov` | 帧门期望水平 FOV(°) | 25.0 | (1,179)；与 `fovDegrees` 一致 | 1030, 1075-1076 |
| `--shutter-speed` | 相机快门 | 120.0 | 写入 applySettings | 1031 |
| `--iso` | 相机 ISO | 600.0 | 写入 applySettings | 1032 |
| `--arm-length` | 相机吊臂 cm | **0.0** | 哨兵第一人称必须为 0（收 400cm Tower 吊臂） | 1033-1034 |
| `--max-fps` | UE 渲染/共享帧上限（`t.MaxFPS`） | 60.0 | 有限且 ≥0，0=不限；CPU 推理抢资源时降 30 | 1035-1037, 1079-1080, tutorial:296-298 |
| `--frame-codec` | 像素无损编码 | **lz4**（stdio 启动器改为 raw） | `raw`/`lz4`；LZ4 无缩小时自动逐帧回退 raw | 1038-1040, 884-893 |
| `--no-fire` | 强制所有 RBExtAim fire=0 | 关 | 调试瞄准方向时使用 | 1041-1042 |
| `--mapping-timeout` | 等待共享帧 mapping 秒数 | 15.0 | 游戏未带 `-visionbridge` 会超时 | 1043 |
| `--command-timeout` | 命令通道静默清火秒数 | 0.5 | 超时即发 fire=0 | 1044 |
| `--no-identity-gate` | 跳过接管身份帧校验 | 关 | **不安全**，仅调试 | 1045-1046 |

### 2.2 代理内部协议常量与固定行为（不可经 CLI 设置，调试需知）

| 项 | 值 | 含义 | 出处（行号） |
|---|---|---|---|
| `ENVELOPE_MAGIC` / `ENVELOPE_VERSION` | `HFGNET1\0` / 1 | WSL↔代理信封 | 27-28 |
| `MESSAGE_FRAME` / `MESSAGE_COMMAND` | 1 / 2 | 信封类型 | 29-30 |
| `FRAME_MAPPING_VERSION` | **3**（shared-frame v3） | 共享内存协议版本；region header 4096B、slot header 256B | 35-38 |
| SHM mapping 名 | `{47534652-414D-4501-0000-%012X}`（%012X=游戏PID） | 进程私有，避免多开串帧 | 231 |
| `GESTALT_PROCESS_NAMES` | `robotbridgedemo-win64-{shipping,development}.exe` 等 | auto 端口发现的进程名白名单 | 48-52 |
| `PIXEL_FORMAT_*` | BGRA8=1 / RGBA8=2 / A2B10G10R10=3；LZ4 偏移 +100 | 线上格式 | 64-67 |
| claim 续租周期 | 1.0 s（游戏侧 5s lease 失效） | `claim_interval_s` | 363, 645-647 |
| takeover/camera 重发周期 | 2.0 s（帧契约未过时） | `next_rearm` | 567, 649-653 |
| prep 门限 | MatchStatus 遥测 60s；spawn 15s×3 次；SetMatchStatus 2s×15 次 | 任一失败即安全失败 | 583, 619, 636-641, 663-669 |
| `arm()` 固定序列 | `ExtAimClaim 1` → `RBTakeOver` → 弹量 → 渲染四件套 → `applySettings` | WS（重）连后重挂 | 550-564 |
| 坐标符号变换 | UE yaw° = −hfut yaw_rad·180/π；pitch° 同号 | RH↔LH 手系转换；速度同理 | 751-755 |
| 命令 mode 门 | 仅 `mode==1`（normal_measurement）转发瞄准，否则清火 | GimbalMode 见 §4.4 | 746-757 |
| 帧契约（frame gate） | identity flags `& 0x7 == 0x7`、player 一致、map≥0、`view_actor==takeover_target`、epoch≠0、1280x720、FOV±0.1、arm±0.1cm；**同一非零 epoch 连续 3 帧**才放行并（可选）开赛 | 839-858, 942-961 |
| 退出序列 | fire=0 → `ExtAimClaim 0` → `RBTakeOver release` | 正常退出归还瞄准权 | 700-714 |

---

## 3. WSL 启动器 `tools/gestalt_stdio_launcher.py`

### 3.1 argparse 参数（`gestalt_stdio_launcher.py:23-48`）

| 参数 | 含义 | 默认值 | 取值建议或约束 | 出处（行号） |
|---|---|---|---|---|
| `--windows-python` | Windows Python 启动器路径 | `$GESTALT_WINDOWS_PYTHON` 或 `/mnt/c/Windows/py.exe` | 须能从 WSL interop 执行 | 26-28 |
| `--debug` | 透传给 `run.sh`/`bringup_sim`（imshow + PlotJuggler） | 关 | — | 29, 105-106 |
| `--allow-fire` | 允许开火（默认强制 fire=0 的语义） | 关 | ⚠ **当前无效**：转发 `--no-fire` 的代码已被注释（87-88行），且 `scripts/run_gestalt_stdio.sh:5` 恒附加 `--debug --allow-fire`——实际链路默认**允许开火**，与 tutorial:191 "默认禁止开火" 的表述不符，以代码为准 | 30-32, 87-88 |
| `--max-fps` | 透传代理 `--max-fps` | 60.0 | 见 §2.1 | 33 |
| `--frame-codec` | 透传代理编码 | **raw** | stdio 带宽足够（interop ≈328MiB/s），先用 raw 避免编码 CPU 开销；`lz4` 可选 | 34-36, tutorial:264-270 |
| `--game-pid` | 透传代理 `--game-pid` | 0 | 多开游戏时指定 | 37, 83-84 |
| `--endpoint-timeout` | 透传代理 | 60.0 | — | 38 |
| `--entity-id` | 透传代理 `--entity-id` | **66000001（英雄）** | ⚠ 与代理自身默认 66000005（哨兵）不同——经 stdio 链路默认生成的是英雄；要哨兵 HACHISEN 须显式 `--entity-id 66000005` | 39 |
| `--team-id` | 透传代理 `--team-id` | 0 | — | 40 |
| `--allowance` | 透传代理 `--allowance` | 400 | — | 41 |
| `--existing-match` | 跳过 prepare/start-match，附着已有可控玩家 | 关 | 不加时自动追加 `--prepare-match --start-match` | 42-44, 85-86 |
| `bringup_args`（`--` 后） | 透传 `scripts/run.sh` | — | 如 `--detector=traditional` 等 | 45-47, 107-110 |

### 3.2 启动器固定透传（不可改，硬编码）

- 代理侧：`auto --transport stdio --player-id 0 --frame-width 1280 --frame-height 720 --fov 25 --shutter-speed 120 --iso 600 --arm-length 0`（gestalt_stdio_launcher.py:71-78）
- 算法侧：`scripts/run.sh --gestalt --gestalt-read-fd=<proxy stdout> --gestalt-write-fd=<proxy stdin>`，经 `pass_fds` 传递，interop 管道不经过 IP/TCP（99-116）
- 环境变量：`GESTALT_WINDOWS_PYTHON`（gestalt_stdio_launcher.py:27）

---

## 4. 算法侧（`configs/simulation.yaml` + `io/gestalt`）

### 4.1 `configs/simulation.yaml` 的 `bridge.gestalt` 段（bringup_sim.cpp:638-697 读取）

| 名称/层级 | 含义 | 默认值 | 取值建议或约束 | 出处 |
|---|---|---|---|---|
| `bridge.gestalt.host` | Windows 代理 TCP 地址 | `""` | 空 ⇒ `$GESTALT_BRIDGE_HOST` ⇒ `127.0.0.1`；WSL2 NAT 用默认路由网关地址；stdio 模式不使用 | simulation.yaml:42, bringup_sim.cpp:639-643 |
| `bridge.gestalt.port` | 代理 TCP 端口 | 47000 | [1,65535]，越界抛异常 | simulation.yaml:43, bringup_sim.cpp:685-686 |
| `bridge.gestalt.player_id` | 期望接管玩家 | 0 | WSL 端逐帧校验 `takeover_player_id` 一致，不符即断开 | simulation.yaml:44, gestalt_bridge_client.cpp:236 |
| `bridge.gestalt.enemy_color` | 检测器敌方颜色 | `"blue"` | HACHISEN 属红方 ⇒ 敌为蓝；仅 gestalt path 生效，webots 保持红色默认 | simulation.yaml:46-49, bringup_sim.cpp:648-657,721 |
| `bridge.gestalt.sim_quad_scale` | PnP 前装甲四点尺度修正 | **1.192** | 有限且 >0；仅作用于 PnP 求解副本，不改检测框 | simulation.yaml:54, bringup_sim.cpp:692-693,940 |
| `bridge.gestalt.forward_auto_aim_velocity` | 普通瞄准是否转发速度前馈 | **false** | 输出滤波器按 250Hz 反算速度，而 Gestalt 命令率≈渲染帧率（默认 60），直接转发会过大；普通瞄准只发绝对角，巡扫仍发自带前馈 | simulation.yaml:56-61, bringup_sim.cpp:976 |
| `bridge.gestalt.idle_scan.enabled` | 无目标巡扫开关 | true | 仅替换 `no_valid_measurement` 输出；TRACKING/TEMP_LOST 不受影响 | simulation.yaml:67 |
| `…idle_scan.activation_delay_s` | 无目标持续多久后启动巡扫 | **0.5** | 代码结构体默认 3.0（被 yaml 覆盖） | simulation.yaml:69, gestalt_idle_scanner.hpp:16 |
| `…idle_scan.yaw_rate_deg_s` | yaw 扫描速率 | **120.0** | 代码默认 20.0；按真实 steady_clock 积分 | simulation.yaml:70, gestalt_idle_scanner.hpp:13 |
| `…idle_scan.pitch_rate_deg_s` | pitch 扫描速率 | **30.0** | 代码默认 6.0；三角波 | simulation.yaml:71, gestalt_idle_scanner.hpp:14 |
| `…idle_scan.pitch_limit_deg` | pitch 扫描带 ±° | **10.0** | 代码默认 30.0；带外平滑回带 | simulation.yaml:72, gestalt_idle_scanner.hpp:15 |
| `…idle_scan.max_step_s` | 单步积分上限 | 0.20 | 防卡帧跳变（120°/s×0.2s≤24°/帧） | simulation.yaml:73-74, gestalt_idle_scanner.hpp:17 |

> ⚠ 文档口径漂移：bridge-doc:66-69 与 tutorial:318-340 写 yaw 60/pitch 20/±10/延迟 3s，
> 与当前 yaml（120/30/±10/0.5s）不一致，以 `simulation.yaml` 为准。
> 相关跟踪参数：`tracker.yaml` 的 `observation_noise_scale: 0.35`、`max_temp_lost_prediction_s: 0.15`（bringup_sim.cpp:389-390）。

### 4.2 同文件其他 gestalt 相关段

| 名称/层级 | 含义 | 值 | 出处 |
|---|---|---|---|
| `camera_to_barrel.gestalt.xyz/rpy` | 相机→枪管外参（m / rad） | `[0.1, 0.05, 0.0505]` / 全 0；`armLength=0` 对应 HACHISEN 枪口视角 | simulation.yaml:15-18 |
| `controller.bullet_speed.gestalt` | 弹速覆盖（m/s） | 24.35（Cal17mm 真值 24.2-24.5） | simulation.yaml:21-25 |

### 4.3 环境变量与 `bringup_sim` 命令行

| 名称 | 含义 | 默认/约束 | 出处 |
|---|---|---|---|
| `GESTALT_BRIDGE_HOST` | 代理地址（yaml host 为空时生效） | `run_gestalt.sh`：mirrored 网络⇒`127.0.0.1`；NAT⇒默认路由网关 | bringup_sim.cpp:641-642, scripts/run_gestalt.sh:6-18 |
| `GESTALT_WINDOWS_PYTHON` | Windows py.exe 路径 | 默认 `/mnt/c/Windows/py.exe` | gestalt_stdio_launcher.py:27-28 |
| `--gestalt` | bringup_sim 开关：bridge_path=gestalt + input_mode=vision | 由 run_gestalt.sh / 启动器附加 | bringup_sim.cpp:324-326 |
| `--gestalt-read-fd=` / `--gestalt-write-fd=` | stdio 传输的文件描述符 | 必须成对出现，否则抛异常；TCP 模式不需要 | bringup_sim.cpp:339-342, 688-690 |

### 4.4 `io/gestalt` 协议常量（WSL 端校验，不可配置）★

| 项 | 值 | 含义/约束 | 出处 |
|---|---|---|---|
| `kEnvelopeMagic` / `kProtocolVersion` | `HFGNET1\0` / 1 | 与代理信封一致 | gestalt_protocol.hpp:11-12 |
| `kDefaultPort` | 47000 | — | gestalt_protocol.hpp:13 |
| `kMaxFramePayload` | 64 MiB | 帧负载上限 | gestalt_protocol.hpp:14 |
| `PixelFormat` | bgra8=1 / rgba8=2 / a2b10g10r10=3；lz4 变体 101/102/103 | 与代理 `PIXEL_FORMAT_LZ4_OFFSET=100` 对应 | gestalt_protocol.hpp:23-30 |
| `Envelope` / `FrameMetadata` | 32 B / **148 B**（`#pragma pack(1)`，小端） | 含 seq、capture_time_s(QPC)、world_time、宽高、FOV、armLength、相机位置/四元数、writer_pid、view_actor、takeover_target/player/map/epoch、identity_flags | gestalt_protocol.hpp:34-73 |
| 命令包 | magic `HFUTCMD1`、version 1、**112 B** CommandPacket | rad/rad/s 上行，代理转度 | io/bridge_protocol.hpp:28-29,122-146 |
| `GimbalMode` | blind=-2 / no_valid=-1 / unknown=0 / **normal_measurement=1** | 代理仅 mode==1 转发瞄准+开火 | io/bridge_protocol.hpp:42-47, gestalt_bridge_windows.py:747 |
| WSL 帧校验 | seq 递增、`identity_flags & 0x7 == 0x7`、`view_actor == takeover_target ≠ 0`、player 一致、FOV∈(1,179)、row_bytes=width*4 | 任一失败即断开重连 | gestalt_bridge_client.cpp:224-242 |
| 内参 | `fx=fy=width/2/tan(FOV/2)`，主点帧中心，零畸变 | 每帧按实际 FOV 计算 | gestalt_bridge_client.cpp:324-336 |

---

## 5. 关键约束速查 ★

### 5.1 第一视角归属
- 生成：`Respawn 0 66000005 0`（player 0 / 哨兵 HACHISEN 66000005 / 红方 team 0）。
- 身份四重验证：PlayerID=0、TeamID=0、Class=**1004**、ConnectionEntityConfigId=**66000005**，且 HP>0、map id 更新（防老尸体快照）。
- 帧侧归属：identity flags **0x7** 全 1、`view_actor == takeover_target`、非零 takeover epoch 连续 3 帧。
- 敌我颜色：红方哨兵 ⇒ `enemy_color: blue`。

### 5.2 相机参数
- `enabled=1, fovDegrees=25, shutterSpeed=120, iso=600, armLength=0`。
- `armLength:0` 是关键：哨兵 Tower 预设 400cm 吊臂=第三人称，会破坏枪口外参；复活/重建 pawn 后回落默认必须重新 apply。
- applySettings 为合并语义（未写键沿用上次）。

### 5.3 帧协议
- shared-frame **v3**（mapping version 3；region header 4096B，slot header 256B；mapping 名含游戏 PID）。
- 帧门：**1280x720**、**FOV 25°**、armLength 0、identity flags 0x7；同一非零 epoch 连续 3 帧才放行/开赛。
- 像素：raw BGRA/RGBA/A2B10G10R10 或 **LZ4 无损**（格式 id +100；无缩小自动回退 raw）；**不发 JPEG**（有损会改灯条/关键点）。
- 信封 `HFGNET1\0` v1 + 148B FrameMetadata；WSL 端重复同等校验。

### 5.4 网络/部署
- 游戏 WS 端口动态随机：`ws_port=auto` 按进程名+WS 握手发现；Shipping 读 `.wsport` sidecar；不要查固定 9002。
- 主路径 **WSL interop stdio**（≈328 MiB/s，不经过 IP/防火墙）；**TCP 回退**（mirrored ≈93 MiB/s）。
- WSL2 **mirrored**：`GESTALT_BRIDGE_HOST=127.0.0.1`；**NAT**（默认）：代理须 `--listen 0.0.0.0`，WSL 用默认路由网关 IP，`--allow-client` 限制来源；防火墙仅专用网络放行 TCP 47000，不配路由器端口转发。
- 失效安全：代理 1s claim 续租（游戏 5s lease）、0.5s 命令静默清火（游戏另有 0.75s fire watchdog）、退出时 `ExtAimClaim 0` + `RBTakeOver release`。

### 5.5 已知文档/代码不一致（以代码为准）
1. `run_gestalt_stdio.sh:5` 恒附加 `--debug --allow-fire`，且启动器转发 `--no-fire` 的代码已注释（gestalt_stdio_launcher.py:87-88）→ **stdio 链路实际默认允许开火**；tutorial:191 "默认禁止开火" 已过时。
2. stdio 启动器 `--entity-id` 默认 **66000001（英雄）**，覆盖代理默认 66000005（哨兵）；要 HACHISEN 必须显式传 `--entity-id 66000005`。
3. `idle_scan` 当前 yaml 值（120/30/±10/0.5s）与 bridge-doc/tutorial 文档值（60/20/±10/3s）不一致，以 yaml 为准。
