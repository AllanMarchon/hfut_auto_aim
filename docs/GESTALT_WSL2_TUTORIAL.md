# Gestalt Steam + hfut_auto_aim 启动教程

本文只针对当前机器的实际环境：

- Windows 上只有 Steam 安装的 Gestalt。
- 游戏目录为 `E:\SteamLibrary\steamapps\common\GestaltSystem`。
- WSL 发行版为 `Ubuntu22.04`。
- WSL interop 已启用；`mirrored` 网络仅供 TCP 回退路径使用。
- `hfut_auto_aim` 位于 WSL 的 `/root/hfut_auto_aim`。

不需要另行获取 `gestalt_system` 开发仓库，也不需要
`ai-match-selftest.ps1`、`rb-console-exec.ps1` 或固定的 `9002` 端口。

桥的二进制协议和坐标系见 [GESTALT_BRIDGE.md](GESTALT_BRIDGE.md)。

## 1. 先解释 reference 的启动流程

reference 教程默认操作者同时拥有两份内容：

1. Gestalt 的 Windows 游戏包。
2. Gestalt 主开发仓库中的 PowerShell 辅助脚本。

其中：

- `ai-match-selftest.ps1` 只是游戏启动包装器。它组装 `-autostart` 参数、创建日志和
  `.wsport` sidecar，并在脚本终端中等待游戏进程。
- `-SkipMatchStart 1` 是上述脚本的参数，不是游戏参数。它的实际效果是不向游戏追加
  `SetMatchStatus 1`，让游戏停在 prep。
- `-MatchObserveSeconds` 也是脚本的等待时间，不是游戏参数。
- `rb-console-exec.ps1` 是一个 WebSocket 命令客户端，用于发送 `Respawn`、
  `SetAttribute` 和 `SetMatchStatus`。
- reference 的 `gestalt.exe` 同时包含视觉算法、AttributeMap 编排和完整对局裁决。

你的 Steam 目录已经包含真正需要的 Shipping 游戏代码。当前包的
`manifest.json` 显示它是 `v0.1.9.69`、`shipping-public` 的 CI 发布包，且包内
`StartGame.js` 支持教程所需的 `-autostart`、`-mapid`、`-nettype`、
`-externalvisioncontrol` 和 `-visionbridge`。缺少的是开发仓库的辅助脚本，不是另一个
游戏或驱动。

本仓库的 Windows 代理已经代替这些脚本完成：

- 自动查找游戏随机选择的 WebSocket 端口。
- 在 prep 中发送 `Respawn 0 66000005 0`。
- 从 AttributeMap 验证 player、team、class、entity 和 HP。
- 设置内置底盘 AI、初始弹量、claim、takeover 和相机。
- 帧契约通过后再发送 `SetMatchStatus 1`。

## 2. 实际数据路径

```text
Windows Gestalt Shipping
  |-- 随机回环 WebSocket --> Windows Python 代理
  |-- 按游戏 PID 命名的 shared-frame v3 --> Windows Python 代理
                                      |
                                      | stdout: frame envelope + pixels
                                      | stdin:  command envelope
                                      | WSL interop binary pipes (no TCP/IP)
                                      v
                          Ubuntu22.04 / hfut_auto_aim
```

游戏 WebSocket 必须留在 Windows 回环地址。WSL 不直接连接这个随机端口；Windows
代理读取它和 Windows 共享内存，由 WSL 启动器把代理的 stdin/stdout 直接接到 C++。

## 3. 一次性准备

### 3.1 确认 Steam 游戏文件

在 Windows PowerShell 中执行：

```powershell
$GameRoot = 'E:\SteamLibrary\steamapps\common\GestaltSystem'
$GameExe = Join-Path $GameRoot 'Gestalt_System.exe'

Test-Path $GameExe
Get-Content (Join-Path $GameRoot 'manifest.json') -Raw
```

`Test-Path` 必须输出 `True`。注意 `$GameExe` 必须指向 exe，不能只填目录。

### 3.2 确认 WSL interop

在 WSL 中执行：

```bash
cat /proc/sys/fs/binfmt_misc/WSLInterop
/mnt/c/Windows/py.exe -c "print('Windows Python interop OK')"
```

预期分别看到 `enabled` 和：

```text
Windows Python interop OK
```

stdio 主路径不依赖 WSL 网络模式，不使用 NAT 网关、`47000`、Windows 防火墙规则或
`netsh interface portproxy`。只有 TCP 回退路径需要 `mirrored`。

### 3.3 安装 Windows 代理依赖

在 Windows PowerShell 中执行：

```powershell
$Distro = 'Ubuntu22.04'
$HfutRepo = "\\wsl.localhost\$Distro\root\hfut_auto_aim"

Test-Path $HfutRepo
py --version
py -m pip install -r "$HfutRepo\tools\requirements-gestalt-windows.txt"
```

`Test-Path` 必须输出 `True`。

### 3.4 构建 WSL 程序

在 WSL 中执行：

```bash
cd /root/hfut_auto_aim
./scripts/build.sh
test -x build/bringup_sim && echo 'bringup_sim OK'
```

## 4. 每次启动只需要两个终端

| 终端 | 环境 | 用途 |
| --- | --- | --- |
| A | Windows PowerShell | 启动并承载 Steam 游戏 |
| B | WSL `Ubuntu22.04` | 通过 interop 管道启动 Windows 代理和 `hfut_auto_aim` |

严格按 A、B 的顺序启动。同一时间只保留一个
`RobotBridgeDemo-Win64-Shipping.exe`。

## 5. 终端 A：直接启动 Steam 游戏并停在 prep

先正常关闭已有 Gestalt 窗口。然后在 Windows PowerShell 执行：

```powershell
$GameRoot = 'E:\SteamLibrary\steamapps\common\GestaltSystem'
$GameExe = Join-Path $GameRoot 'Gestalt_System.exe'

if (-not (Test-Path $GameExe)) {
  throw "找不到游戏：$GameExe"
}

$GameArgs = @(
  '-autostart'
  '-mapid=4'
  '-nettype=0'
  '-hudhidden=1'
  '-execdelay=15000'
  '-externalvisioncontrol'
  '-visionbridge'
  '-wsbind=127.0.0.1'
  '-windowed'
  '-ResX=1280'
  '-ResY=720'
)

& $GameExe @GameArgs
```

这个 PowerShell 会一直等待游戏退出，运行期间不要关闭终端 A。

这里特意没有加入：

```text
-exec="SetMatchStatus 1"
```

所以游戏自动进入地图 4 后保持 `MatchStatus=0`。Windows 代理完成生成车辆、接管和帧
验证前，比赛不会开始。

三个 Shipping 授权参数必须保持原样：

```text
-externalvisioncontrol -visionbridge -wsbind=127.0.0.1
```

`-externalvisioncontrol` 必须是裸参数，不能写成 `-externalvisioncontrol=1`。

## 6. 终端 B：通过 WSL interop 管道启动代理和算法

在 `Ubuntu22.04` 中执行：

```bash
cd /root/hfut_auto_aim
./scripts/run_gestalt_stdio.sh --debug
```

该命令当前**默认允许开火**（`run_gestalt_stdio.sh` 恒附加 `--allow-fire`，
启动器转发 `--no-fire` 的代码已被注释；GESTALT_PARAMS.md §259 以代码为准），
并完成以下工作：

1. 由 WSL 直接启动 `C:\Windows\py.exe` 和 Windows 代理。
2. 找到 `RobotBridgeDemo-Win64-Shipping.exe` 拥有的回环 listener。
3. 对候选端口做 WebSocket 握手，排除同进程的非 WS listener。
4. 用同一个游戏 PID 打开 shared-frame v3 mapping。
5. 订阅 AttributeMap，并确认当前仍处于 prep。
6. 生成并验证红方 HACHISEN：player `0`、team `0`、class `1004`、entity
   `66000005`、HP 大于 `0`。
7. 设置 `IsAI=1` 和 400 发初始 allowance，再执行 claim、takeover、60 FPS 渲染上限和
   相机设置。
8. 将 Windows 代理 stdout 直接接到 C++ 收帧文件描述符，将 C++ 命令文件描述符接到
   代理 stdin；图像和命令均不经过 TCP/IP。
9. 启动 `bringup_sim`，warmup 模型后进入 latest-frame 闭环。

预期先后看到类似输出：

```text
[gestalt-win] discovered game WebSocket: 127.0.0.1:<随机端口> pid=<游戏PID>
[gestalt-win] game WS connected :<随机端口>; waiting for prep telemetry
[gestalt-win] prep: Respawn 0 66000005 0
[gestalt-win] prep passed: player=0 map=<map> team=0 class=1004 entity=66000005; takeover armed
[gestalt-win] game pid=<游戏PID>; WSL interop stdio; fire=disabled; max_fps=60; codec=raw
[bringup_sim] bridge path: gestalt
[bringup_sim] gestalt transport: WSL interop stdio player=0
```

帧契约通过后还会输出：

```text
[gestalt-win] frame contract passed: 1280x720 fov=25.00 arm=0.00cm pid=0 map=<map> epoch=<epoch>
[gestalt-win] frame gate passed; sending SetMatchStatus 1
[gestalt-win] match start confirmed: MatchStatus=1
```

`--start-match` 的含义仍是“帧契约通过后开赛”，不是启动代理就立即开赛。代理要求同一
非零 takeover epoch 下连续三帧同时满足：

- `1280x720`。
- 水平 FOV 为 `25` 度。
- `armLength=0`，即枪管第一人称相机。
- identity flags bit 0..2 全部有效。
- takeover player 为 `0`，map 有效。
- ViewActor 等于 takeover target。

任一条件不满足时，代理不会向 WSL 放行帧，也不会执行 `SetMatchStatus 1`。

### TCP 兼容回退

只有在 Windows interop 被系统策略禁用时才使用旧 TCP 路径。先在 Windows PowerShell
运行代理：

```powershell
$HfutRepo = '\\wsl.localhost\Ubuntu22.04\root\hfut_auto_aim'
py "$HfutRepo\tools\gestalt_bridge_windows.py" auto `
  --prepare-match --start-match --listen 127.0.0.1 `
  --max-fps 60 --frame-codec lz4 --no-fire
```

再在 WSL 中运行：

```bash
cd /root/hfut_auto_aim
./scripts/run_gestalt.sh --debug
```

### 发布率、控制率与性能

共享帧发布器在 UE 每次完成渲染帧时写入一个新槽，本身没有固定 30 Hz 的协议限制。
reference 主动执行 `t.MaxFPS 30`，因为其 CPU OpenVINO 推理在游戏以 60 FPS 渲染时会
受到 CPU 竞争，实测单帧一度约 110 ms。当前桥默认改为 `--max-fps 60`，更适合配置中
CUDA/FP16 后端；`--max-fps 0` 可解除上限，但通常没有必要无限制发布。

stdio 一键启动器默认使用 `--frame-codec raw`，性能不依赖压缩。当前机器上 Windows
Python 向 WSL 连续写入 128 MiB 的实测结果为：interop pipe 约 `0.39 s / 328 MiB/s`，
mirrored TCP 约 `1.38 s / 93 MiB/s`。因此主路径绕过 IP/TCP 后，1280x720 BGRA 的
约 211 MiB/s 峰值流量仍在管道能力内。

LZ4 仍可通过 `./scripts/run_gestalt_stdio.sh --frame-codec lz4` 启用。它不改变灯条颜色或
角点，压缩后没有变小时会自动回退 raw；但 stdio 下应先用 raw 测试，避免编码 CPU 开销。
WSL 后台收帧线程持续接收、解压并完成 BGRA 转 BGR，主线程只取得最新完整帧，因此传输
处理会与当前帧推理并行，旧帧不会排队。

发布率不等于最终控制率。当前 WSL 后台线程持续收帧，主线程对最新帧执行推理、PnP、
跟踪和控制；实际命令率仍是最新帧到达率与算法处理吞吐的较低值。终端会直接给出：

```text
[gestalt-win] ... publisher_fps=60.0 forwarded_fps=59.8 ratio=2.40x encode=2.1ms send=5.3ms
[bringup_sim] ... source=60.0Hz loop=55.2Hz process=17.1ms infer=10.4ms dropped=18
```

- `publisher_fps`：游戏真正产生新渲染帧的速度。
- `forwarded_fps`：Windows 代理成功写入 interop pipe 或 TCP 的速度。
- `ratio` / `encode` / `send`：无损压缩率、编码耗时和管道/TCP 写入阻塞耗时；raw 时
  `ratio` 应接近 `1.0x`。
- `source`：主线程实际消费的相邻最新帧时间戳频率。
- `loop`：检测到控制命令的完整循环频率。
- `process` / `infer`：整帧处理和纯模型推理耗时。
- `dropped`：后台线程收到新帧时，主线程尚未消费的旧帧累计覆盖数；覆盖是限延迟行为。

外部 pipeline 配置中的 `controller.control_rate: 250` 来自 ROS 节点版本；当前
ROS-free `bringup_sim` 没有据此创建 250 Hz 控制定时器，而是每个新图像帧计算一次命令。
同一配置中的 `output_filter.one_euro_freq: 250` 仍会用于输出滤波器的导数换算，这正是
Gestalt 普通瞄准默认不转发该速度值的原因。这里的 `250` 不能当成当前实测控制率。

若 `publisher_fps` 约 60 而 `loop` 明显较低，瓶颈在 WSL 算法或数据搬运；先确认没有
回退到 CPU，再根据 `infer` 和 `process` 判断。若使用 CPU 后端且游戏与推理互相争抢，
将代理参数改为 `--max-fps 30` 是有依据的降载手段。

### 相机参数、颜色与控制输出

Gestalt 不复用 Webots 相机参数。每帧携带实际分辨率与水平 FOV，WSL 按针孔模型计算
`fx=fy=width/(2*tan(FOV/2))`；在 1280x720、25 度时约为 `2886.9`，主点为
`(640, 360)`，畸变为零，与 reference 配置一致。PnP 前还只对求解副本应用
`sim_quad_scale: 1.192`，补偿 Gestalt 渲染装甲四点的尺度，不改变检测框。

准备出的 HACHISEN 属于红方，所以 `bridge.gestalt.enemy_color: blue` 只在 Gestalt 路径
启用，Webots 仍保持原来的红色敌方默认值。普通 Gestalt 自瞄按 reference 只发送绝对
角度；pipeline 输出滤波器按 250 Hz 固定频率反算出的速度不再送进游戏，避免实际帧率
不是 250 Hz 时产生过大的速度前馈。无目标巡扫仍发送自身按真实时间积分的低速前馈。

当前 pipeline 配置的控制策略是 `predicted`，不是 `mpc`。因此 `controller.mpc` 下的
`N/dt/Q/R` 暂时不参与命令计算；若以后切换为 `mpc`，当前桥会把每帧实际相机半视场
同步给 MPC 的 FOV 约束。

### Gestalt 无目标巡扫

`hfut_auto_aim` 收到 Gestalt 帧后，如果 pipeline 持续 3 秒没有有效自瞄命令，会自动进入巡扫：

- yaw 以 `120 deg/s` 连续旋转。
- pitch 以 `30 deg/s` 在 `-10 deg` 到 `+10 deg` 之间做三角波扫描。
- 每帧积分最多使用 `0.20 s`，避免模型或画面卡顿后突然大角度跳转。
- `TRACKING` 始终由自瞄控制；`TEMP_LOST` 预测最多保留 `0.15s`，超时即清空控制目标。
- 完全无目标后先保持 0.5 秒，再由巡扫接管；短暂漏帧不会直接触发扫描。
- 检测器刚看到装甲但 tracker 尚未建轨时，巡扫立即冻结在当前实际角度。
- 巡扫命令始终强制 `fire=0`。

参数位于 `configs/simulation.yaml`（源码当前生效值如下；旧文档中的
3s、60/20 deg/s 已过时）：

```yaml
bridge:
  gestalt:
    idle_scan:
      enabled: true
      activation_delay_s: 0.5
      yaw_rate_deg_s: 120.0
      pitch_rate_deg_s: 30.0
      pitch_limit_deg: 10.0
      max_step_s: 0.20
```

tracker 预测输出和观测权重位于同一配置的 `tracking` 段：

```yaml
tracking:
  observation_noise_scale: 0.35
  max_temp_lost_prediction_s: 0.15
```

终端 B 会在状态切换时输出：

```text
[bringup_sim] gestalt idle scan started
[bringup_sim] gestalt idle scan stopped
```

## 8. 从无火测试切换到正式开火

注意：stdio 一键启动当前**默认允许开火**（启动器转发 `--no-fire` 的代码已
注释，`run_gestalt_stdio.sh` 恒附加 `--allow-fire`）——先检查检测框、yaw 和
pitch 方向时务必确认预期；需要禁火时先检查脚本行为再操作。开火状态由以下
命令控制：

```bash
./scripts/run_gestalt_stdio.sh --debug --allow-fire
```

TCP 回退模式则从 Windows 代理命令中删除 `--no-fire`。

不要在已经开赛的旧局中直接重跑 `--prepare-match`；编排器会拒绝
`MatchStatus>=1` 的晚接管。

## 9. 为什么不再查询固定的 9002

游戏会扫描可用端口并选择一个 WebSocket listener，所以每次运行的端口可能不同。
以下命令报“找不到对象”只说明当时没有进程监听 `9002`，不代表 CIM 或 WSL 故障：

```powershell
Get-NetTCPConnection -State Listen -LocalPort 9002
```

正常流程完全不需要手工查询端口。诊断时应按游戏 PID 查看全部 listener：

```powershell
$GameProcess = Get-Process 'RobotBridgeDemo-Win64-Shipping' |
  Sort-Object StartTime -Descending |
  Select-Object -First 1

Get-NetTCPConnection -State Listen -OwningProcess $GameProcess.Id `
  -ErrorAction SilentlyContinue |
  Format-Table LocalAddress, LocalPort, OwningProcess
```

代理还会对这些候选端口执行真实 WebSocket 握手，因此不需要人来猜哪一个是 WS。

## 10. 正确停止顺序

1. 在 WSL 终端 B 按 `Ctrl+C`。启动器先停止 `bringup_sim`，管道 EOF 随后让 Windows
   代理清火并释放接管。
2. 等待终端 B 返回后，关闭 Gestalt 窗口，终端 A 随游戏一起返回。

代理正常退出时会依次清火、发送 `ExtAimClaim 0 0`，再发送
`UEExec RBTakeOver release`。游戏侧另有约 0.75 秒 fire watchdog 和约 5 秒 claim
lease，处理进程崩溃或断连。

## 11. 常见错误

### `no usable loopback WebSocket found for the Gestalt process`

依次检查：

1. 终端 A 是否仍在运行。
2. `Get-Process RobotBridgeDemo-Win64-Shipping` 是否能找到且只有一个游戏进程。
3. 启动参数是否包含 `-autostart -mapid=4 -nettype=0 -wsbind=127.0.0.1`。
4. 是否等待了地图加载完成；默认端点发现会等待 60 秒。

### `frame mapping ... is unavailable`

游戏启动时没有带 `-visionbridge`，或者包体不支持 shared-frame v3。当前
`v0.1.9.69 shipping-public` 包应支持；先关闭游戏，再用本文终端 A 命令重启。

### `MatchStatus=1; restart the game in prep`

游戏已经开赛。通常是启动参数中误加了 `-exec="SetMatchStatus 1"`，或者通过 UI 手工
开赛。关闭游戏并按本文终端 A 命令重新启动。

### `red HACHISEN spawn/identity gate failed after 3 attempts`

代理没有同时验证到 player `0`、team `0`、class `1004`、entity `66000005` 和存活
HP。确认地图参数为 `-mapid=4`，且没有第二个游戏实例；不要在运行中通过 UI 更换车辆。

### `waiting for frame contract: player=-1 expected=0`

takeover 尚未落到生成的 player 0。代理会在帧门通过前定期重发 takeover 和相机设置；
若持续超过 30 秒，检查终端 B 更早的 prep 日志。

### `waiting for frame contract: arm=400.00cm expected=0.00cm`

当前仍是 Tower 第三人称吊臂。确认代理命令有 `--arm-length 0`，并检查游戏启动时包含
`-externalvisioncontrol -visionbridge`。

### WSL 持续 `no camera frame`

stdio 模式按顺序检查：

1. `/proc/sys/fs/binfmt_misc/WSLInterop` 是否存在且内容为 `enabled`。
2. `/mnt/c/Windows/py.exe -c "print('ok')"` 是否能从 WSL 正常执行。
3. 终端 B 是否输出 `WSL interop stdio` 和 `gestalt transport: WSL interop stdio`。
4. 是否已经输出 `frame contract passed`；若没有，查看更早的身份门错误。

TCP 回退模式再检查 mirrored 网络、`127.0.0.1:47000` listener、`WSL client connected`
以及端口是否被旧代理占用。

### 有画面但没有检测

网络和游戏桥已经工作。继续检查 ONNX Runtime/CUDA、模型路径、敌方颜色、目标是否在
FOV 内，以及 debug 画面是否确实为枪管第一人称。

## 12. 与 reference 完整 match 模式的差异

本流程已经复刻启动所需的随机端口发现、prep、生成身份门、claim/takeover、相机帧门
和帧门后开赛，足够让 Steam-only 环境启动 `hfut_auto_aim` 闭环。

它尚未完整复刻 reference `gestalt.exe` 的双轴物理响应证明、每次死亡后的 AttributeMap
重建裁决、全局 WS generation 证明和自然结算 `RESULT=MATCH_COMPLETE` 判定。因此
`frame contract passed` 只能证明当前像素、相机和接管身份一致，不能等价为 reference 的
完整对局验收结果。

## 13. 不启动游戏的离线自检

在 WSL 中执行：

```bash
cd /root/hfut_auto_aim
cmake --build build --target gestalt_bridge_client_test gestalt_idle_scanner_test
ctest --test-dir build --output-on-failure -R 'gestalt_(bridge_client|idle_scanner)'
python3 -m unittest discover \
  -s tests \
  -p 'test_gestalt_bridge_windows_protocol.py' \
  -v
```

这些测试覆盖 TCP 与 stdio 分帧、结构尺寸、raw/LZ4 无损像素、latest-frame 覆盖、后台
收帧与命令并发、FOV 内参、坐标变换、命令分片、yaw/pitch 角度与速度回程符号、自动端口
参数解析、Steam prep 身份门，以及巡扫速度、卡帧步长和 pitch 双侧边界。
