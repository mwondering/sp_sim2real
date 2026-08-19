# G1 机载部署总说明

本文统一说明当前仓库在 G1 机载计算机上运行策略推理、遥操作重定向和底层 bridge 的方式。
“机载部署”表示实时计算闭环不依赖外部 PC；PICO、身体追踪器、G1 实体遥控器以及 PICO 到 G1
的无线局域网仍然是系统组成部分。仓库当前没有提供 systemd 开机自启单元，默认仍需通过 SSH
或机载终端启动和查看进程。

## 1. 当前支持边界

| 任务 | 全机载状态 | 外部设备 | 说明 |
| --- | --- | --- | --- |
| 通用 `deploy.py` tracking + PICO | 已有完整代码与启动链路 | PICO、trackers、G1 遥控器、Wi-Fi/LAN | 推荐作为首个机载验证任务 |
| Dual-Teacher `motion` 模式 | 链路可同机运行，尚缺机载负载实测 | PICO、trackers、G1 遥控器、Wi-Fi/LAN | 不需要深度相机；adapter 会强制使用 VR motion source |
| `teleop-upper-lower-locomani` | 实验性全机载流程，尚未验证时延和真机闭环 | PICO、trackers、本地外接 D435i、G1 遥控器 | 当前已确认手册仍以外部策略服务器为主 |
| TAP—terrain + 可切换 teleop | 实验性全机载流程，尚未验证时延和真机闭环 | 本地外接 D435i、G1 遥控器；teleop 时另需 PICO、trackers、Wi-Fi/LAN | 不使用宇树内置相机；默认预加载 `0818_terrain` 与 `0819_teleop`，两个 PICO primary 组合键双向切换 |

任何“链路可同机运行”都不等价于对应 checkpoint 已通过真机验证。首次上机必须先完成 sim2sim，
使用吊架或保护绳，并由操作者始终握住实体遥控器和硬件急停。

## 2. 机载通信拓扑

通用 PICO 遥操作链路为：

```text
PICO/控制器
  │ XRoboToolkit 网络流，Wi-Fi/LAN
  ▼
G1 机载计算机
  XRoboToolkit PC Service
  │ 本机 SDK 回调
  ▼
  serve_xrobot_teleop.py
  │ ZMQ 127.0.0.1:28701/28702/28703
  ▼
  deploy.py / task adapter / ONNX Runtime
  │ UDP 127.0.0.1:55001/55002
  ▼
  g1_udp_bridge
  │ Unitree DDS，G1_NET=eth0
  ▼
  rt/lowstate + rt/lowcmd
```

`28701` 用于策略请求参考帧，`28702` 返回重定向后的 root 和关节参考，`28703` 发送 PICO
控制器状态。`55001` 从 bridge 向策略发送关节、IMU、力矩、诊断和 G1 遥控器状态；`55002`
从策略向 bridge 发送 `q_des`、`qd_des`、`kp`、`kd` 和控制使能。全机载时 ZMQ connect 地址、
bridge 状态目标和策略命令目标都应使用回环地址，不需要填写外部策略机 IP。通用
`g1_bridge.yaml` 的命令 socket 默认绑定 `0.0.0.0`；在不可信网络中，应复制 bridge 配置并把
`udp.cmd_bind_host` 限制为 `127.0.0.1`，或至少设置 `udp.cmd_allowed_host: "127.0.0.1"`。

## 3. 首次安装

### 3.1 复制运行时与 checkpoint

把仓库完整复制到 G1 机载计算机，至少保留：

```text
<repo>/sim2real
<repo>/g1_sim2real
<repo>/ckpts                  # 仅所选策略引用这里时需要
```

检查所选 YAML 的 `policy_path`、任务 ONNX 和 JSON sidecar。当前仓库内默认 PMG、compliance、
SPV5-2、TAP terrain/teleop、Dual-Teacher 和 MJLab locomani checkpoint 可解析；`tracking_wbteleop.yaml` 与
`tracking_spv5_1.yaml` 指向的专用 checkpoint 目录当前未随仓库提供，不能直接照抄对应启动命令。

### 3.2 安装 Python 和 XR 依赖

通用 Python 运行时要求 Python 3.10：

```bash
cd <repo>/sim2real
uv sync
```

使用 PICO 的任务还必须在 G1 上安装 XRoboToolkit ARM64/headless PC Service，并在完成 `uv sync`
后构建 Python binding：

```bash
cd <repo>/sim2real
bash install_xrobottoolkit_sdk.sh
```

完整安装、ABI 风险和验证命令见 [`sim2real/teleop/README.md`](sim2real/teleop/README.md)。
TAP—terrain 和 upper/lower locomani 的 D435i 发送端使用独立 `g1-camera-stream` 环境；不要把
其 Python 3.12/`pyrealsense2` 环境与主策略的 Python 3.10 环境混用。TAP 只运行 terrain 时
不要求 XR 服务；需要切换到 `0819_teleop` 时，则必须同时完成本节的 PICO/XR 安装。这里的
D435i 是通过 USB 接入的本地外接深度相机：相机分支按 `camera.yaml` 中的序列号直接读取其
RealSense 内参和深度尺度，不能替换成宇树内置头部相机、其 `camera_info` 或图像 topic。

TAP 的相机参数以任务配置为准：`640×360@30 Hz`、目标 FOV
`89.041605°×57.9°`，相对 pelvis 位移
`[0.125461581205,0.0197,-0.059520040551] m`，TAP `+X` 前向射线坐标系四元数
`wxyz=[0.86601716,0.00377874,0.49999524,-0.00218165]`。机载启动前必须把实际 D435i 序列号写入
`/home/unitree/g1-camera-stream/camera.yaml`，并使用
`depth_camera_sender.py --config camera.yaml` 启动。

### 3.3 在 G1 上重新编译 bridge

仓库中的已有 build 产物可能来自 x86-64 主机，不能复制后直接运行。必须在 G1 上重新构建：

```bash
cd <repo>/g1_sim2real
G1_BRIDGE_BUILD_DIR=build_onboard bash scripts/build.sh
file build_onboard/g1_udp_bridge
```

`file` 输出必须表明该二进制为 AArch64/ARM64。仓库内 vendored Unitree SDK 同时提供
`aarch64` 静态库，构建系统会按当前架构选择。

### 3.4 配置网络

1. 用 `ip -br address` 确认 G1 的 DDS 网卡和可被 PICO 访问的无线 IP。
2. bridge 的 `G1_NET` 应填写 DDS 网卡名，机载默认示例为 `eth0`，不是机器人 IP。
3. 在 PICO XRoboToolkit 客户端中填写 G1 的无线 IP。
4. 确保 PICO 与 G1 处于可互通的低时延局域网；放行 XRoboToolkit 所需流量。
5. 不要把代理或 VPN 路由到 `127.0.0.1`，必要时将回环地址加入 `no_proxy`。

## 4. 通用 tracking + PICO 最小流程

### 4.1 创建机载 VR 配置

不要覆盖原 UDP 配置，复制所选、且 checkpoint 完整的 tracking YAML。例如先使用默认 PMG：

```bash
cd <repo>/sim2real
cp config/g1/tracking.yaml config/g1/tracking_onboard_vr.yaml
```

只把新文件中的 motion source 改为：

```yaml
motion_source:
  type: "vr"
```

保留 `vr.req_addr/rep_addr/ctrl_addr` 的 `127.0.0.1` 地址。若复制的是其他 actor 配置，先确认
其 ONNX、JSON 以及可能的 `.onnx.data` 都已放到机载侧。

### 4.2 启动顺序

终端 1，启动 XRoboToolkit PC Service，然后在 PICO 上连接 G1 无线 IP并完成标定：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

终端 2，启动仅绑定回环地址的 PICO 重定向服务：

```bash
cd <repo>/sim2real
taskset -c 1 bash scripts/run_pico_server.sh
```

终端 3，启动 G1 bridge：

```bash
cd <repo>/g1_sim2real
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_onboard \
taskset -c 2-3 bash scripts/run_bridge.sh
```

终端 4，启动策略：

```bash
cd <repo>/sim2real
taskset -c 4-7 uv run src/deploy.py \
  --robot g1 \
  --no-record \
  --tracking-config tracking_onboard_vr.yaml
```

G1 机载 CPU 资源有限；上面的绑核只是起点，不是所有任务的性能保证。首次验证建议关闭非必要
可视化和记录，并持续观察策略周期、ZMQ frame age、UDP 丢包和 PICO stale 日志。

### 4.3 进入控制与验收

1. bridge 必须持续收到 LowState，策略必须打印已连接 robot state。
2. 遥操作终端必须打印三条 ZMQ endpoint；策略必须打印 `VRMotionSource` 已连接。
3. 按 G1 遥控器 `start` 进入默认姿态，稳定后按 G1 遥控器 `A` 进入 tracking。
4. 保持自然站姿，按 PICO 右手 `A` 启动或重新对齐姿态流。
5. 先做小幅、低速动作，确认参考、关节方向、延迟和停止链路均正确。
6. 正常停止使用 G1 遥控器 `select/stop`；异常时优先使用硬件急停。

满足以下条件后，才能认为通用机载链路完成基本验收：持续运行期间没有 `pico_stale` 或 bridge
state timeout；策略周期满足 50 Hz；PICO、策略、bridge 任一进程退出时，现场人员能立即通过
实体停止装置终止运动。需要深度的任务还必须分别满足其文档中的相机新鲜度与有效率要求。

## 5. 任务专用入口

- Dual-Teacher：见 [`dual-teacher-deployment_zh.md`](dual-teacher-deployment_zh.md) 的“G1 机载部署”。
- upper/lower locomani 真机：见 [`teleop-upper-lower-locomani-sim2real_zh.md`](teleop-upper-lower-locomani-sim2real_zh.md) 的“全机载部署”。
- TAP—terrain：见 [`tap-terrain-deployment_zh.md`](tap-terrain-deployment_zh.md) 的“G1 全机载部署”。
- PICO/XR 依赖与联通排障：见 [`sim2real/teleop/README.md`](sim2real/teleop/README.md)。

## 6. 已知边界

- 当前仓库没有提供机载 systemd、自恢复或进程监督配置；“无需外部计算主机”不等于已经实现
  无人值守开机启动。
- 通用 `g1_bridge.yaml` 的电机诊断是只读报警，不会因诊断超限自动进入 damping；软件停止不能
  替代硬件急停。
- 裸命令 `serve_xrobot_teleop.py --robot g1` 默认配置可绑定 `tcp://*`。机载部署应优先使用
  `scripts/run_pico_server.sh`，其 `teleop-server.yaml` 只绑定 `127.0.0.1`。
- 当前全机载性能和真机闭环结论只能按任务分别给出；不要把通用 tracking 的可用性外推到
  Dual-Teacher、upper/lower locomani 或 TAP—terrain。
