# teleop-upper-lower-locomani：完整启动与操作手册

本文档说明如何在 G1 sim2sim 中启动并操作 `teleop-upper-lower-locomani`。该任务包含两种运行模式：

- 全身遥操作模式：使用 PICO 全身参考，可在启动时选择 HEFT 或 SPV5-2。
- 上下肢分离模式：上肢跟随 PICO，下肢使用 MJLab 带深度地形 locomotion 策略。

右手 B 只负责在上述两种模式间切换。HEFT 和 SPV5-2 不能在运行中互相切换，必须在启动任务时选择。

本文主体是 sim2sim 手册。真机和实验性全机载流程见
[teleop-upper-lower-locomani 真机启动与操作](teleop-upper-lower-locomani-sim2real_zh.md)，通用
机载拓扑与各任务成熟度见 [G1 机载部署总说明](onboard-deployment_zh.md)。

## 1. 当前实现与操作边界

- 仅支持 G1。
- MuJoCo 物理频率为 200 Hz，策略频率为 50 Hz，深度更新频率为 30 Hz。
- PICO 数据沿用原 ZMQ 端口：28701、28702、28703。
- 相机是独立进程。sim2sim 通过 28810 发布仿真状态，相机通过 28811 发布深度。
- 深度不再单独弹出 Matplotlib 窗口，而是在主 MuJoCo 窗口显示为彩色点云。
- 当前基座高度固定为 0.65 m，摇杆不控制高度。
- 楼梯模式只允许向前运动，不允许横移和转向。
- 平地/斜坡模式提供 `wz`，但纯原地旋转能力没有经过充分训练，必须在仿真中谨慎测试。

## 2. 目录与环境准备

所有项目命令均从以下目录执行：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
```

首次使用或修改依赖后，执行：

```bash
uv sync
bash install_xrobottoolkit_sdk.sh
```

`xrobottoolkit_sdk` 是手动编译安装包，`uv sync` 可能将它移除。因此顺序必须是先执行 `uv sync`，再执行 `install_xrobottoolkit_sdk.sh`；不要在安装 SDK 后再次执行 `uv sync`。

验证 Python 环境和 PICO SDK：

```bash
uv run python - <<'PY'
import xrobotoolkit_sdk as xrt
import mujoco
import onnxruntime

print("xrobotoolkit_sdk:", xrt.__file__)
print("callback API:", all(
    hasattr(xrt, name)
    for name in (
        "register_frame_callback",
        "clear_frame_callback",
        "has_frame_callback",
    )
))
print("mujoco:", mujoco.__version__)
print("onnxruntime:", onnxruntime.__version__)
PY
```

正常结果必须包含：

```text
callback API: True
```

检查上下肢 ONNX：

```bash
test -f config/g1/ckpts/MJLab_Locomani/lower.onnx
test -f config/g1/ckpts/MJLab_Locomani/upper.onnx
```

两个命令无输出且退出码为 0，表示文件存在。

## 3. 选择测试组合

启动前必须让 bridge 场景和 `--terrain-class` 一致。

| 测试目标 | bridge 配置 | 任务参数 |
| --- | --- | --- |
| 楼梯 + HEFT | `bridge_teleop_upper_lower_locomani.yaml` | `--whole-body-policy heft --terrain-class 2` |
| 楼梯 + SPV5-2 | `bridge_teleop_upper_lower_locomani.yaml` | `--whole-body-policy spv5_2 --terrain-class 2` |
| 斜坡 + HEFT | `bridge_teleop_upper_lower_locomani_slope.yaml` | `--whole-body-policy heft --terrain-class 1` |
| 斜坡 + SPV5-2 | `bridge_teleop_upper_lower_locomani_slope.yaml` | `--whole-body-policy spv5_2 --terrain-class 1` |

建议首次运行使用“楼梯 + HEFT”，确认完整链路后再测试 SPV5-2 和斜坡。

## 4. 完整启动顺序

需要五个终端。启动过程中不要关闭已经运行的终端。

### 终端 1：XRoboToolkit 系统服务

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

然后打开 PICO 上的 XRoboToolkit 应用，连接当前电脑，确认头显、控制器和身体追踪器已经连接并完成校准。

### 终端 2：PICO 重定向服务

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run python teleop/serve_xrobot_teleop.py --robot g1
```

该进程负责：

- 从 XRoboToolkit 读取 PICO 数据；
- 将人体动作重定向为 G1 关节参考；
- 在 28701、28702、28703 上提供姿态和按键数据。

如果终端打印：

```text
viewer_url: http://localhost:8080
```

可以用浏览器打开该地址检查 PICO 重定向姿态。这个网页显示的是重定向结果，不是深度。

### 终端 3：带地形 sim2sim

楼梯：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/sim2sim.py --robot g1 \
  --bridge-config config/g1/bridge_teleop_upper_lower_locomani.yaml
```

斜坡：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/sim2sim.py --robot g1 \
  --bridge-config config/g1/bridge_teleop_upper_lower_locomani_slope.yaml
```

正常启动时应看到类似日志：

```text
[G1Sim2Sim] freq: physical_hz=200, state_decimation=4, state_hz=50.000
[G1Sim2Sim] sim-state stream bind=tcp://*:28810
[DepthOverlay] depth<-tcp://127.0.0.1:28811, points=144
Waiting for high level controller...
```

此时 MuJoCo 窗口已经打开，但高层任务尚未连接。

### 终端 4：独立相机进程

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/depth_camera.py \
  --config config/g1/teleop-upper-lower-locomani.yaml
```

正常日志：

```text
[DepthCamera] state<-tcp://127.0.0.1:28810 depth->tcp://*:28811
```

启用训练形式的距离噪声和 1% dropout：

```bash
uv run src/depth_camera.py \
  --config config/g1/teleop-upper-lower-locomani.yaml \
  --depth-noise
```

首次调试建议不加 `--depth-noise`。

`--show-depth` 仍可保留在旧命令中，但现在只是兼容参数；深度统一显示在主 MuJoCo 窗口。

### 终端 5：双模式部署任务

楼梯 + HEFT：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/teleop_upper_lower_locomani.py \
  --whole-body-policy heft \
  --terrain-class 2
```

楼梯 + SPV5-2：

```bash
uv run src/teleop_upper_lower_locomani.py \
  --whole-body-policy spv5_2 \
  --terrain-class 2
```

斜坡 + HEFT：

```bash
uv run src/teleop_upper_lower_locomani.py \
  --whole-body-policy heft \
  --terrain-class 1
```

斜坡 + SPV5-2：

```bash
uv run src/teleop_upper_lower_locomani.py \
  --whole-body-policy spv5_2 \
  --terrain-class 1
```

正常启动时应看到：

```text
Successfully connected to the robot.
Enter zero torque state.
Waiting for the start signal...
```

这里的 “robot” 指 sim2sim bridge，不是实机。

## 5. 从等待状态进入控制

以下 `s`、`a`、`x` 都是在运行 `sim2sim.py` 的终端中，用电脑键盘输入。必须先点击该终端，让终端获得键盘焦点。不要在 MuJoCo 图形窗口里输入。

### 第一步：电脑键盘按 `s`

在终端 3，即运行 `sim2sim.py` 的终端中按：

```text
s
```

作用：

- 退出零力矩等待状态；
- 部署任务将机器人平滑移动到策略默认姿态；
- sim2sim 使用临时基座稳定器保持机器人站立。

等待机器人姿态稳定，并观察终端日志。正常情况下会出现：

```text
Default target is stable; grounded PD hold is active.
Press 'a' to release the temporary base stabilizer and enter policy
```

### 第二步：电脑键盘按 `a`

仍在终端 3 中按：

```text
a
```

作用：

- 释放临时基座稳定器；
- 启动全身策略；
- sim2sim 进入正式控制循环。

任务终端应出现类似：

```text
Initial policy: tracking
[TeleopLocomani] running: right A=start PICO, right B=switch policy, left X=software stop (damping + task exit)
```

### 第三步：PICO 右手控制器按 A

保持自然站立姿势，按 PICO 右控制器实体 A 键。

任务终端应依次出现：

```text
[VRMotionSource] VR start requested from control button
[VRMotionSource] VR start acknowledged
```

这一步启动 PICO 姿态流并建立初始朝向对齐。重新按右手 A 会重新开始一次 PICO 会话和对齐。

### 第四步：确认全身模式

任务默认从全身模式启动。此时机器人应根据 PICO 全身参考运动，下肢摇杆速度尚未接管机器人。

### 第五步：PICO 右手控制器按 B

按 PICO 右控制器实体 B 键，切换到上下肢分离模式。

正常日志：

```text
[TeleopLocomani] switched -> upper-lower-locomani
```

再次按右手 B，切回全身模式：

```text
[TeleopLocomani] switched -> whole-body
```

模式切换只响应按键从松开到按下的瞬间。切换前先松开全部摇杆并保持至少 0.25 秒，再按 B；机器人倾斜或转动过快时也会拒绝切换。关节目标、KP 和 KD 会在 1.0 秒内平滑过渡。

## 6. 输入设备与按键总表

### 电脑键盘：只在 sim2sim 终端输入

| 按键 | 输入位置 | 功能 |
| --- | --- | --- |
| `s` | 运行 `sim2sim.py` 的终端 | 从零力矩状态进入默认姿态 |
| `a` | 运行 `sim2sim.py` 的终端 | 释放稳定器并进入策略控制 |
| `x` | 运行 `sim2sim.py` 的终端 | 停止 sim2sim 控制流程 |

### PICO 实体控制器

| 按键/摇杆 | 功能 |
| --- | --- |
| 右手 A | 启动或重新对齐 PICO 姿态流 |
| 右手 B | 切换全身模式与上下肢分离模式 |
| 左手 X | 软件停止：发送 damping 并结束高层策略任务 |
| 左摇杆 Y | 前后速度 `vx` |
| 左摇杆 X | 横向速度 `vy` |
| 右摇杆 X | 角速度 `wz` |

电脑键盘的 `x` 和 PICO 左手 X 仍是两条独立停止路径：

- 电脑键盘 `x`：停止 sim2sim 控制流程。
- PICO 左手 X：高层任务发送 damping 后退出；随后 sim2sim 因命令中断停止控制。

## 7. 两种策略模式

### 全身模式

全身模式使用启动时选择的策略：

- `--whole-body-policy heft`：加载 `tracking.yaml` 中的 HEFT/G1_PMG 策略。
- `--whole-body-policy spv5_2`：加载当前 `tracking_spv5_2.yaml`。

全身模式读取完整 PICO 姿态参考。即使上下肢分离模式正在输出控制，全身策略仍保持更新，便于平滑切回。

### 上下肢分离模式

- 上肢：读取 PICO 14 关节参考及 torso 坐标系下的双腕末端位姿。
- 下肢：读取左、右摇杆速度命令、机器人状态、历史观测和相机深度。
- 深度输入必须保持新鲜；相机超时阈值为 0.25 秒。
- PICO 或深度失效时，任务进入锁定的 `safe-hold`，不会在行走中自动切到另一策略。

如果按右手 B 时条件尚未满足，任务终端会打印：

```text
[TeleopLocomani] right B ignored: wait for active PICO pose and a fresh camera frame
```

此时应先确认：

1. 已按 PICO 右手 A；
2. 相机进程正在运行；
3. 相机终端没有异常；
4. sim2sim 和相机使用同一任务 XML。

## 8. 摇杆和地形约束

### 斜坡或平地：`terrain-class=0/1`

- 左摇杆 Y：`vx`，范围约为 ±1.0。
- 左摇杆 X：`vy`，范围约为 ±0.5。
- 右摇杆 X：`wz`，范围约为 ±0.5。
- 摇杆死区为 0.1。
- 基座高度固定为 0.65 m。

策略接口支持转向，但没有专门的纯角速度训练桶。因此普通行走转向可以测试，严格原地旋转的稳定性尚未确认。

### 楼梯：`terrain-class=2`

- 只允许向前运动；
- 前进命令最小为 `vx=0.3`；
- 左摇杆 Y 中立或向后时按停止处理；
- 强制 `vy=0`；
- 强制 `wz=0`；
- 不支持在楼梯上横移、倒退或原地旋转。

楼梯测试时，即使移动左摇杆 X 或右摇杆 X，也不会产生横移或转向，这是预期行为。

## 9. MuJoCo 深度点云

深度链路为：

```text
sim2sim
  └─ qpos/qvel，tcp://127.0.0.1:28810
       └─ depth_camera
            ├─ [1,36,64] 策略深度，tcp://127.0.0.1:28811
            ├─ 下肢 ONNX
            └─ sim2sim 主 MuJoCo 窗口彩色点云
```

点云含义：

- 蓝色：较近；
- 绿色/黄色：中等距离；
- 红色：较远；
- 点云位置跟随 pelvis 上的 `depth_camera` 挂载点；
- 当前默认每隔 4 个像素采样一次，共显示 144 个点。

点云是策略实际预处理深度的稀疏三维表示，不是额外渲染相机。策略仍接收完整的 `[1,36,64] float32` 深度。

调整显示密度和点大小：

```yaml
depth_debug:
  enabled: true
  connect: "tcp://127.0.0.1:28811"
  site_name: "depth_camera"
  stride: 4
  point_size: 0.012
```

该配置位于：

- `config/g1/bridge_teleop_upper_lower_locomani.yaml`
- `config/g1/bridge_teleop_upper_lower_locomani_slope.yaml`

减小 `stride` 会增加点数和显示开销；增大 `point_size` 会让点更明显。

## 10. 正常停止顺序

推荐按以下顺序停止：

1. 按 PICO 左手 X，让高层任务发送 damping 并退出。
2. 在运行 sim2sim 的终端按电脑键盘 `x`，结束仿真。
3. 在任务终端按 `Ctrl+C`。
4. 在相机终端按 `Ctrl+C`。
5. 在 PICO 重定向终端按 `Ctrl+C`。
6. 最后停止 XRoboToolkit 系统服务。

如果某个进程没有响应，可以在对应终端按一次 `Ctrl+C`，等待其清理端口后再重新启动。

## 11. 快速检查运行状态

检查相关进程：

```bash
pgrep -af \
  'serve_xrobot_teleop.py|sim2sim.py|depth_camera.py|teleop_upper_lower_locomani.py'
```

检查 TCP 端口：

```bash
ss -ltnp | rg '28701|28702|28703|28810|28811'
```

预期：

- 28701、28702、28703：PICO 重定向服务；
- 28810：sim2sim 状态流；
- 28811：相机深度流。

重新验证 SDK：

```bash
uv run python - <<'PY'
import xrobotoolkit_sdk as xrt
print(xrt.__file__)
print("callback API:", hasattr(xrt, "register_frame_callback"))
PY
```

## 12. 常见问题

### 12.1 `Failed to import 'xrobotoolkit_sdk'`

先安装系统编译依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  libprotobuf-dev protobuf-compiler \
  libgrpc++-dev libgrpc-dev protobuf-compiler-grpc
```

然后重新安装 SDK：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
bash install_xrobottoolkit_sdk.sh
```

如果刚执行过 `uv sync`，也必须重新执行安装脚本。

### 12.2 MuJoCo 窗口没有深度点云

依次检查：

1. 重启 sim2sim，使其加载最新的 `depth_debug` 配置；
2. 确认相机进程正在运行；
3. sim2sim 终端应有 `[DepthOverlay] ... points=144`；
4. 相机终端应有 `[DepthCamera] state<-... depth->...`；
5. 确认使用的是本任务的 bridge，而不是原 `bridge.yaml`；
6. 检查 28810 和 28811 是否被正确监听。

相机进程可以在任务策略之前启动，但 sim2sim 必须已经启动并发布状态。

### 12.3 按右手 B 没有切换

如果看到：

```text
right B ignored: wait for active PICO pose and a fresh camera frame
```

说明 PICO 姿态或相机深度尚未就绪。先按右手 A，确认相机进程运行，再松开并重新按右手 B。

### 12.4 进入 `safe-hold`

如果看到：

```text
dual input stale; entering safe hold
```

说明 PICO 姿态或相机深度超过 0.25 秒没有更新。检查相机进程、PICO 网络和 CPU 负载。输入恢复后，先让所有摇杆保持中立，再按右手 B，可回到全身模式；也可直接停止任务。

### 12.5 楼梯上无法转向

这是训练约束，不是按键故障。楼梯 `terrain-class=2` 强制 `vy=0、wz=0`。需要测试转向时，使用斜坡 bridge，并把任务参数改为 `--terrain-class 1`。

### 12.6 `Address already in use`

通常是旧进程没有退出。先检查：

```bash
pgrep -af \
  'serve_xrobot_teleop.py|sim2sim.py|depth_camera.py|teleop_upper_lower_locomani.py'
```

回到对应终端按 `Ctrl+C`，确认旧进程退出后再启动。不要同时启动两份相机或两份 sim2sim。

### 12.7 任务一直打印 `no bridge state for 1s`

说明任务没有收到 sim2sim 的 UDP 状态。确认：

- sim2sim 正在运行；
- sim2sim 使用本任务 bridge；
- 没有另一份 sim2sim 占用 55001/55002；
- 启动任务后，sim2sim 终端出现 `Connected to high level`。

### 12.8 PICO 姿态不更新

检查：

- XRoboToolkit 系统服务是否运行；
- PICO 应用是否连接当前电脑；
- 身体追踪和控制器数据是否可用；
- `serve_xrobot_teleop.py` 是否持续运行；
- 是否已经按 PICO 右手 A；
- 浏览器 `http://localhost:8080` 中的重定向机器人是否运动。

## 13. 最小启动命令汇总

以下是“楼梯 + HEFT”的最小命令集合。

终端 1：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

终端 2：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run python teleop/serve_xrobot_teleop.py --robot g1
```

终端 3：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/sim2sim.py --robot g1 \
  --bridge-config config/g1/bridge_teleop_upper_lower_locomani.yaml
```

终端 4：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/depth_camera.py \
  --config config/g1/teleop-upper-lower-locomani.yaml
```

终端 5：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/teleop_upper_lower_locomani.py \
  --whole-body-policy heft \
  --terrain-class 2
```

最后按以下顺序操作：

```text
电脑键盘：sim2sim 终端按 s
电脑键盘：姿态稳定后，在同一终端按 a
PICO 右手控制器：按 A 启动姿态流
PICO 右手控制器：按 B 进入上下肢分离模式
PICO 左摇杆 Y：控制楼梯前进
PICO 右手控制器：再次按 B 返回全身模式
PICO 左手控制器：按 X 发送 damping 并结束高层任务
```

## 14. G1 机载实机部署入口

机载实机不运行本页的 MuJoCo `sim2sim.py` 和仿真 `depth_camera.py`。其进程替换关系为：

| 本页 sim2sim 组件 | 机载实机组件 |
| --- | --- |
| `sim2sim.py` | `g1_sim2real/g1_udp_bridge`，通过 `G1_NET=eth0` 连接 DDS |
| 仿真 `depth_camera.py` | G1 上独立 `g1-camera-stream/depth_camera_sender.py`，端口 28811 |
| 工作站 PICO 服务 | G1 ARM64 XRoboToolkit PC Service + `scripts/run_pico_server.sh` |
| 仿真 task | `teleop_upper_lower_locomani.py --target real` |

全机载时 XR 服务、D435i、PICO 重定向、bridge 和策略全部运行在 G1，PICO 只通过无线局域网
连接 G1。ZMQ `28701-28703`、深度 `28811` 和 UDP `55001/55002` 均使用 localhost；不再填写
外部策略机 IP。推荐的核心起始分配为：D435i `0`、PICO 重定向 `1`、bridge `2-3`、策略
`4-7`。

当前该组合尚未完成机载并发负载和真机闭环验证，不能从本页 sim2sim 结果推断全机载可用。
首次部署、完整五终端命令、验收和回退条件必须按
[真机手册第 10 节](teleop-upper-lower-locomani-sim2real_zh.md) 执行；出现持续 PICO/深度 stale、
bridge state timeout 或控制周期抖动时，应退回外部策略服务器方案。
