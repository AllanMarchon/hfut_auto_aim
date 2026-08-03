# Gestalt bridge

## 提取边界

`reference/sp_vision_25_gestalt_system_bridge` 的 Gestalt 接入分成三层：

1. `shared_frame_capture` 从游戏进程私有 shared-frame v3 读取最终 viewport，且像素与 `frame id / QPC / world time / camera pose / FOV / takeover identity` 属于同一次原子提交。
2. `game_link` 连接游戏本机 WebSocket，读取属性并发送 `RBTakeOver / ExtAimClaim / RBExtAim / rgbCamera.applySettings`。
3. `gestalt.cpp` 把上述 IO 与 sp_vision 检测、PnP、跟踪、控制以及具体比赛编排组合起来。

本仓库以头两层的通用边界为主。Windows 代理负责进程私有资源和游戏命令，
`bringup_sim` 的 `gestalt` path 负责 hfut 检测到控制主链。为了让只有 Steam 游戏包的
环境可以启动，代理的可选 `--prepare-match --start-match` 模式还提取了最小赛前编排：
生成并验证 player 0、设置 AI/弹量，并在帧门通过后开赛。reference 中的双轴物理效果
证明、死亡/复活裁决、整局 bench 和真值统计没有进入 hfut 自瞄进程。

```text
Windows Gestalt process
  shared-frame v3 (process-scoped, frame atomic)
             |
             v
tools/gestalt_bridge_windows.py ---- game loopback WebSocket
             |
             | WSL interop stdout (preferred) or TCP fallback
             | raw/LZ4-lossless pixels + exact camera metadata
             v
WSL2 latest-frame receiver -> detector -> PnP -> tracker -> controller
             |
             | WSL interop stdin or TCP CommandPacket (radians)
             v
Windows proxy -> UEExec RBExtAim (degrees)
```

## 数据约束

- `ws_port=auto` 时，Windows 代理按 Gestalt 进程名筛选 IPv4 回环 listener，并以
  WebSocket 握手确认端口；随后只打开
  `{47534652-414D-4501-0000-%012X}` 对应的进程私有 mapping，避免多开游戏时串帧。
- 共享内存版本固定为 v3，region header 为 4096 B，slot header 为 256 B。读取 slot 前后各检查一次偶数 commit sequence，防止撕裂。
- frame payload 是 148 B `FrameMetadata` 加原始或 LZ4 无损的
  BGRA/RGBA/A2B10G10R10 像素。不发送 JPEG，避免有损压缩改变装甲灯条和关键点；LZ4
  没有缩小时逐帧自动回退 raw。
- 帧必须满足 identity flags `0x7`、player id 一致、view actor 等于 takeover target。WSL 客户端再次执行相同校验。
- `camera_position_ue_cm` 只保留在协议诊断字段中，不作为世界真值注入。和 reference 一样，控制帧原点取相机/云台中心，`t_cam2world = 0`。

## 坐标与单位

Gestalt/UE world 是 `X forward, Y right, Z up` 左手系；hfut control world 是 `X forward, Y left, Z up` 右手系；OpenCV camera 是 `x right, y down, z forward`。

WSL 客户端使用：

```text
R_camera_to_control = diag(1,-1,1)
                    * R_ue_camera_quaternion
                    * [[0,0,1], [1,0,0], [0,-1,0]]
```

内参由每帧的水平 FOV 计算，`fx = fy = width/2/tan(FOV/2)`，主点是完整 shared frame 的中心。算法命令在 pipe/TCP 上保持 rad/rad/s；Windows 回程转换为：

```text
UE yaw degrees   = -hfut yaw radians * 180/pi
UE pitch degrees =  hfut pitch radians * 180/pi
```

速度前馈使用相同符号变换。Gestalt path 在 pipeline 输出
`no_valid_measurement` 时可启用独立巡扫器；源码当前生效 yaw 为 120 deg/s，
pitch 为 30 deg/s，pitch 扫描带为 `[-10,+10]` deg，无目标 0.5 s 后开始巡扫
（旧文档中的 60/20 deg/s、3 s 已过时）。`TRACKING` 不受巡扫覆盖；`TEMP_LOST`
预测最多输出 0.15 s，超时后清空控制目标。原始视觉检测出现但尚未
建轨时使用反馈角零速冻结。

## WSL2 传输

主路径由 WSL 启动 Windows Python 子进程，并把代理 stdout/stdin 作为二进制帧/命令
管道直接传给 C++ 文件描述符。它使用 WSL interop 跨越 Windows/WSL2 边界，不经过
IP/TCP。当前机器 128 MiB 基准中 interop pipe 约 328 MiB/s，mirrored TCP 约 93 MiB/s。
stdio 启动器因此默认发送 raw；LZ4 仅作为可选项。

WSL 后台线程持续排空 pipe/TCP，并用单槽 latest-frame 缓冲覆盖未消费旧帧，避免传输等待
与推理串行相加。TCP 保留为 interop 被禁用时的兼容回退；`/mnt/c` 文件映射仍不采用，
因为 DrvFS 的缓存一致性和轮询语义不适合帧原子提交。

TCP 回退首选 Windows 11 mirrored networking：Windows 与 WSL2 可用
`127.0.0.1:47000` 互连。默认 NAT 模式下，`scripts/run_gestalt.sh` 仍会自动取得 Windows
host IP。Microsoft 的 WSL 网络说明见
[Accessing network applications with WSL](https://learn.microsoft.com/en-us/windows/wsl/networking)。

NAT 下监听 `0.0.0.0` 会让 Windows 把连接视为 LAN 流量。代理仍会把来源限制为 `127.0.0.1` 与 `wsl hostname -I` 的结果；使用非默认发行版时传入 `--allow-client <WSL-IP>`。Windows 防火墙只允许专用网络上的 TCP 47000，不要允许公用网络；无需也不应配置路由器端口转发。

## 失效安全

- Windows 代理每秒续租 `ExtAimClaim`，WebSocket 重连后重新执行 takeover、相机设置和 claim。
- `RBExtAim` 只在 `GimbalMode::normal_measurement` 下接受 `fire=1`。其他 mode 保持最近角度并主动清火。
- 500 ms 内没有新命令、pipe/TCP 断开、WSL 进程退出或代理收到 `Ctrl+C` 时，代理发送 `fire=0`。
- 正常退出时代理发送 `ExtAimClaim <player> 0`，把瞄准权还给游戏策略。

## 验证

Linux 回环测试不依赖游戏：

```bash
cmake --build build --target gestalt_bridge_client_test gestalt_idle_scanner_test
ctest --test-dir build --output-on-failure -R 'gestalt_(bridge_client|idle_scanner)'
```

这些测试覆盖 TCP/pipe envelope、148 B metadata、LZ4/BGRA 解码、后台收帧与命令并发、
FOV 内参、UE 零姿态到 control frame 的变换、112 B 命令回程，以及巡扫速度、卡帧步长
和 pitch 边界。

Windows 代理的纯协议与赛前身份门测试：

```bash
python3 -m unittest discover -s tests -p 'test_gestalt_bridge_windows_protocol.py' -v
```
