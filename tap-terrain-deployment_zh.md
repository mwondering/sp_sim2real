# TAP—terrain sim2sim / sim2real 部署说明

最后更新：2026-08-19。

本文描述 `TAP—terrain` 任务在 Unitree G1 上的仿真和真机启动方式。该任务直接运行完整的
`model_18000_play_full.onnx`，外部适配层只负责构造机器人历史、速度指令和深度图，不重复执行
ONNX 内部的 estimator、深度编码器或 decoder。任务还会预加载可选的
`0819_teleop/model_1000_play_full.onnx`；收到 PICO 两个 primary 按键的组合上升沿后，可在 terrain
与全身 teleop policy 之间双向切换。

当前状态：sim2sim 的模型加载、深度链路、相机外参、ONNX 前向和 A 键原子接管已经通过自动化
测试与同模型物理闭环验证；sim2real 接口已经实现，但该 checkpoint 尚未完成 D435i/G1 实机
闭环验证。首次上机必须使用可靠吊架或保护绳，并确保操作者可立即触发实体停止装置。
通用机载拓扑、ARM64 编译和各任务状态矩阵见 [G1 机载部署总说明](onboard-deployment_zh.md)。

## 1. 部署合同

| 项目 | 值 |
| --- | --- |
| 任务名 | `TAP—terrain` |
| 策略输入 | `spv5_2_terrain_observation`，`float32[1,7484]` |
| 策略输出 | `action`，`float32[1,29]` |
| teleop 策略输入 | `observation`，`float32[batch,8809]` |
| 默认模式 | `terrain`；组合键切到 `teleop`，再次按组合键切回 |
| 切换按键 | PICO 两个 primary：SDK 名称 `left_key_one + right_key_one`（当前映射为左 X + 右 A） |
| 切换过渡 | 1.0 秒关节目标和 PD 增益线性混合；按住组合键只切换一次 |
| 控制频率 | 50 Hz |
| 深度输入 | `float32[1,18,32]` |
| 深度预处理 | 36×64 → crop `(18,0,16,16)` → 3×3 reflect Gaussian blur → clamp `[0.1,1.2] m` → 除以 `1.2` |
| 深度超时 | 0.25 秒 |
| 本体历史 | 50 帧；首次接管时旧 49 帧清零、最新帧写入当前状态 |
| 接管条件 | 有效深度、完整观测和首个 policy action 准备完成后才发送控制使能 |
| sim2sim 速度限制 | `|vx|≤1.0 m/s`、`|vy|≤0.5 m/s`、`|wz|≤0.5 rad/s` |
| sim2real 速度限制 | `0≤vx≤1.0 m/s`、`vy=0`、`|wz|≤0.5 rad/s` |

7484 维输入的固定布局为：

```text
[0:805]       decoder proprio_without_estimates
[805:6905]    SPV5.2 estimator 50 帧历史
[6905:6908]   [vx, vy, wz]
[6908:7484]   1x18x32 深度图，按 NCHW 展平
```

## 2. 文件与 checkpoint

| 场景 | 任务配置 | 底层 bridge 配置 |
| --- | --- | --- |
| sim2sim | `sim2real/config/g1/tap-terrain.yaml` | `sim2real/config/g1/bridge_tap_terrain.yaml` |
| sim2real | `sim2real/config/g1/tap-terrain-real.yaml` | `g1_sim2real/config/g1_bridge.yaml` |

### 2.1 地形复用关系

TAP—terrain 不维护另一套楼梯或斜坡几何。仿真任务直接复用已有场景：

```text
sim2real/config/g1/assets/g1_teleop_upper_lower_locomani.xml
```

该 XML 同时被 `teleop-upper-lower-locomani.yaml` 和 `tap-terrain.yaml` 引用，内部已有
`training_terrain`、楼梯平台，以及“上坡、坡顶平台、下行楼梯”组合地形。TAP 在该 XML 中只使用独立的
`tap_terrain_depth_camera` site，以匹配 checkpoint 训练时的相机外参，没有新增地形几何。

`teleop-upper-lower-locomani-real.yaml` 是实机任务配置，仅通过 `kinematics_xml` 引用同一机器人
模型；实机配置本身不包含 MuJoCo `scene_xml`。`bridge_tap_terrain.yaml` 也不生成地形，只负责
sim2sim 的关节映射、初始位置、频率、状态流和调试选项。

策略配置为：

```text
sim2real/config/g1/tap-terrain-policy.yaml
sim2real/config/g1/tap-teleop-policy.yaml
```

checkpoint 默认使用仓库内的可移植副本：

```text
ckpts/0818_terrain/model_18000_play_full.onnx
ckpts/0818_terrain/model_18000_play_full.json
ckpts/0819_teleop/model_1000_play_full.onnx
ckpts/0819_teleop/model_1000_play_full.json
```

`tap-terrain-policy.yaml` 使用相对于自身目录的路径
`../../../ckpts/0818_terrain/model_18000_play_full.onnx` 访问模型。完整复制仓库即可保留该目录关系；
`tap-teleop-policy.yaml` 同样使用相对路径访问 `../../../ckpts/0819_teleop/model_1000_play_full.onnx`。
若只复制部分运行文件，则必须同时复制每个 ONNX 和同名 JSON，并同步修改相应 `policy_path`。

校验文件：

```bash
sha256sum \
  ckpts/0818_terrain/model_18000_play_full.onnx \
  ckpts/0818_terrain/model_18000_play_full.json \
  ckpts/0819_teleop/model_1000_play_full.onnx \
  ckpts/0819_teleop/model_1000_play_full.json
```

预期 SHA-256：

```text
655c27340a8a254e7e0fed8c4857969a3266823464dfcba09f8c631c11ace25a  model_18000_play_full.onnx
8cfe20694b10f42d210bff4a6f0e5dad1fb3d9bed0287c042d1b3b9955dd2201  model_18000_play_full.json
6a62bb28357534eaefd837a47a92c3c1671c413650fef7a9ba62d790a432b1dc  model_1000_play_full.onnx
81f95a810a30a7190b6bf409f03a07780c9df1b0206eb1287273665380ae12d6  model_1000_play_full.json
```

## 3. 公共准备与自检

安装 Python 环境：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv sync
```

运行任务测试：

```bash
uv run python -m unittest tests.test_tap_terrain
```

测试应通过 11 项检查，包括 7484/8809 维布局、训练一致的历史初始化、训练侧深度预处理、精确
相机外参、坡道有效深度、两套真实 ONNX 输入输出与完整前向、组合键边沿，以及 A 键到首个
terrain 策略动作的使能时序。

运行完整回归测试：

```bash
uv run python -m unittest discover -s tests -v
```

当前完整测试集为 71 项。

## 4. sim2sim 启动

sim2sim 共使用三个终端。建议按以下顺序启动。

### 终端 1：MuJoCo bridge

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/sim2sim.py \
  --robot g1 \
  --bridge-config config/g1/bridge_tap_terrain.yaml
```

该配置把机器人放在现有场景 `y=-10` 的坡道通道起点，启用 50 Hz 状态流，并关闭只支持旧
`1×36×64` 合同的点云调试叠加层。它不会创建或复制楼梯/斜坡几何。

### 终端 2：MuJoCo 深度相机

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/depth_camera.py \
  --config config/g1/tap-terrain.yaml
```

正常日志应包含：

```text
output_shape=(1, 18, 32)
```

### 终端 3：TAP—terrain 策略

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run src/tap_terrain.py --target sim
```

默认仿真速度为 `[0,0,0]`，由终端 1 的 WASD/QE 按键实时产生速度指令。需要绕过键盘并使用
固定低速命令时，可以显式传入：

```bash
uv run src/tap_terrain.py --target sim --velocity 0.2 0.0 0.0
```

不需要保存 NPZ 记录时添加 `--no-record`。默认记录目录为：

```text
sim2real/assets/policy_logs/g1/
```

只验证 terrain 时仍然只需上述三个进程。若要验证 policy 切换，还必须先启动 XRoboToolkit PC
Service、让 PICO 连接策略计算机，并另开终端启动重定向服务：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
bash scripts/run_pico_server.sh
```

策略启动日志应同时显示 `terrain_onnx=...0818_terrain...`、
`teleop_onnx=...0819_teleop...` 和三条本机 `28701/28702/28703` ZMQ 地址。XR 服务安装与 PICO
标定步骤见 [`sim2real/teleop/README.md`](sim2real/teleop/README.md)。

### 进入控制

TAP 启动键和运动键均输入到终端 1，而不是策略终端。为避免 `S/A` 与 WASD 冲突，TAP 专用
bridge 将旧启动键重映射为 `R/P/X`。

1. 三个进程均已运行后，在终端 1 按 `r`。
2. 等待机器人进入训练默认姿态，并确认 grounded PD hold 已稳定。
3. 确认机器人初始速度指令为 `[0,0,0]`，然后在终端 1 按 `p`。
4. P 键只选择 TAP 任务；有效深度未到达时，sim2sim 继续保留临时基座稳定器。
5. 策略进程打印 `depth ready` 后，首个策略动作与控制使能在同一帧发送，同时释放稳定器。
6. 机器人应立即由策略接管并保持零速度；按住 WASD/QE 后才产生运动命令，松开后相应轴回零。
7. 已启动 PICO 链路时，同时按下两个 primary 按键（SDK 当前映射为左 X + 右 A）切到 teleop；
   松开后再次同时按下，切回 terrain。按住不放不会连续切换。
8. 切回 terrain 只在 D435i 深度仍新鲜时接受，否则保持 teleop 并打印拒绝原因。
9. 停止时在终端 1 按 `x`，让高层任务发送 damping 并退出。
10. 分别结束 MuJoCo、深度相机、PICO 重定向与 XR 服务进程。

## 5. sim2real 首次准备

### 5.1 物理安全与相机安装

首次真机运行前必须满足：

- G1 使用可靠吊架或保护绳，脚下和前方无人、无松动物体；
- 一名操作者始终握住 G1 实体遥控器，并熟悉硬件停止操作；
- 深度源必须是本地安装的外接 Intel RealSense D435i，不能使用宇树内置头部相机、其
  `camera_info` 或图像 topic；
- D435i 固定在 pelvis；相对位移为 `[0.125461581205,0.0197,-0.059520040551] m`，TAP
  `+X` 前向射线坐标系四元数 `wxyz=[0.86601716,0.00377874,0.49999524,-0.00218165]`；
- 本地 D435i 深度流为 `640×360@30 Hz` 原始 Z16，目标视场角为
  `89.041605°×57.9°`；
- 已经使用低速命令完成 sim2sim 验证。

`tap-terrain.yaml`、`tap-terrain-real.yaml` 和 XML site 已使用同一组 FOV 与外参。本地相机的
`fx/fy/ppx/ppy` 和深度尺度由 `g1-camera-stream` 中的 `pyrealsense2` 直接从所选 D435i 读取，
不是从宇树相机配置读取。真机支架仍需按上述外参安装并现场复测；不能仅凭“能收到图像”判断
相机已经对齐。

### 5.2 编译 G1 bridge

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/g1_sim2real
bash scripts/build.sh
```

如果旧 `build/` 记录了其他源码路径，使用独立构建目录：

```bash
G1_BRIDGE_BUILD_DIR=build_tap_terrain bash scripts/build.sh
```

确认策略计算机连接 G1 的 DDS 网卡名：

```bash
ip -br address
```

### 5.3 准备 D435i 发送端

真机深度发送端使用独立的 `g1-camera-stream` 分支，直接打开本地 USB D435i 并在 G1 机载
计算机发布。该链路与宇树内置相机服务无关：

```text
protocol=d435i-raw-z16-v1
dtype=uint16
shape=(360,640)
endpoint=tcp://*:28811
```

相机分支、Python 3.12 和 `pyrealsense2` 的首次安装步骤见
[`teleop-upper-lower-locomani-sim2real_zh.md`](./teleop-upper-lower-locomani-sim2real_zh.md) 的“首次准备”章节。
相机端只发送原始深度；米制转换、无效值处理、缩放、裁剪、模糊与归一化全部由
`TAP—terrain` 策略进程完成。

部署前先枚举本地 D435i，并把实际序列号写入相机分支的 `camera.yaml`；不要保留空字符串：

```bash
cd /home/unitree/g1-camera-stream
.venv/bin/python -c \
  'import pyrealsense2 as rs; print([(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()])'
```

`camera.yaml` 的相机块应与 TAP 配置一致：

```yaml
hardware:
  width: 640
  height: 360
  fps: 30
  serial_number: "<LOCAL_D435I_SERIAL>"
  expected_fov_x_deg: 89.04160508162592
  expected_fov_y_deg: 57.9
  fov_tolerance_deg: 6.0
```

发送端启动时会从这个序列号对应的 D435i 读取设备内参和 `depth_scale`。外参不来自相机 SDK，
由物理支架和上节给出的 pelvis 安装位姿保证。

## 6. sim2real 启动

真机只运行 terrain 时最少使用三个终端：D435i 发送端、G1 bridge、策略进程，不要求 PICO。
若需要切到 `0819_teleop`，还必须启动 XRoboToolkit PC Service、连接并标定 PICO，再运行
`scripts/run_pico_server.sh`。PICO 重定向服务与策略进程必须使用同一组 `28701/28702/28703`
地址；默认均为本机回环地址。

### 终端 1：D435i 原始深度发送端

在策略计算机设置 G1 机载计算机地址：

```bash
export ROBOT_IP="192.168.123.164"
export ROBOT_SSH="unitree@${ROBOT_IP}"
export G1_CAMERA_ROOT="/home/unitree/g1-camera-stream"

ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py --config camera.yaml"
```

正常日志应包含 `D435i`、`640x360@30`、`raw-z16`、与 `camera.yaml` 相同的实际设备序列号、
实际 FOV 和 `depth_scale`。若临时使用 `--serial-number` 覆盖配置，值也必须是同一台本地 D435i。

### 终端 2：G1 底层 bridge

将 `enp3s0` 替换为实际 DDS 网卡：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/g1_sim2real
G1_NET=enp3s0 \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
bash scripts/run_bridge.sh
```

如果使用独立构建目录：

```bash
G1_NET=enp3s0 \
G1_BRIDGE_BUILD_DIR=build_tap_terrain \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
bash scripts/run_bridge.sh
```

必须确认 bridge 持续收到 LowState，并向 Python 端提供 `tau_latest`。如果策略提示缺少
`tau_latest`，说明 bridge 版本过旧，必须重新编译并重启。

### 终端 3：TAP—terrain 策略

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
export ROBOT_IP="192.168.123.164"
uv run src/tap_terrain.py \
  --target real \
  --depth-connect "tcp://${ROBOT_IP}:28811"
```

首次排查时可添加 `--no-record`；正式验证建议保留默认记录，以便关联策略输入、动作、关节状态、
力矩和电机诊断信息。

### 进入控制

G1 实体遥控器仍负责底层启动和停止；启用 teleop 切换时，PICO 只负责人体参考和高层模式组合
键，不能替代实体停止装置。策略进入前保持 G1 与 PICO 摇杆回中。

1. 确认吊装保护、相机日志、LowState 和 bridge 日志正常。
2. 启动策略进程，确认 ONNX 加载成功且显示 `Action mapping: 29/29 mapped`。
3. 按 G1 遥控器 `start`，机器人在约 2 秒内进入训练默认姿态。
4. 等待机器人完全稳定，再按 G1 遥控器 `A` 进入任务。
5. 保持摇杆回中；`depth ready` 出现后，策略立即接管。
6. 先用很小的左摇杆 Y 正值验证低速直行，再逐步增加；不要首次运行就给满量程。
7. PICO 链路正常时，松开所有摇杆，同时按左、右 primary（当前解析为左 X + 右 A）切到
   teleop；日志必须显示 `switched -> teleop`。
8. 松开组合键后再次同时按下可切回 terrain；只有新鲜深度可用时才接受，日志显示
   `switched -> terrain`。每次切换都执行默认 1 秒目标/增益混合。
9. 发现姿态、深度或动作异常时，立即使用 G1 实体停止装置。

## 7. 速度指令与按键

### 7.1 sim2sim 键盘

TAP 仿真采用按住生效、松开回零的虚拟摇杆。无运动键按下时，策略速度输入严格为
`[vx,vy,wz]=[0,0,0]`。

| 键盘输入 | 策略命令 | 满量程值 |
| --- | --- | --- |
| `W` | `vx` 前进 | `+1.0 m/s` |
| `S` | `vx` 后退 | `-1.0 m/s` |
| `A` | `vy` 左移 | `+0.5 m/s` |
| `D` | `vy` 右移 | `-0.5 m/s` |
| `Q` | `wz` 左转 | `+0.5 rad/s` |
| `E` | `wz` 右转 | `-0.5 rad/s` |
| `R` | 从零力矩进入默认姿态 | 上升沿触发 |
| `P` | 从默认姿态选择并进入策略 | 首个有效 action 到达时使能 |
| `X` | 结束高层任务 | 随后发送一次 damping |

同一轴的相反按键同时有效时相互抵消。终端键盘事件由 `sshkeyboard` 解析，实际松键回零延迟取决于
终端和系统键盘重复参数；控制时应观察终端 1 的 `Key pressed` 日志，并避免失去终端焦点。

注意：该 terrain checkpoint 的训练命令主要覆盖前进和转向，横移与倒退属于训练分布外输入。
接口会按上表生成完整指令，但不能据此推断 A/D/S 在楼梯或斜坡上已经具有可靠闭环稳定性；应先在
平地低风险环境分别验证。

### 7.2 sim2real 遥控器

| 输入 | 策略命令 | 当前限制 |
| --- | --- | --- |
| 左摇杆 Y 正值 | `vx` 前进速度 | `0–1.0 m/s` |
| 左摇杆 Y 负值 | 倒退请求 | 被强制为 `0` |
| 左摇杆 X | `vy` 横移 | 被强制为 `0` |
| 右摇杆 X | `wz` 转向 | `±0.5 rad/s` |
| G1 `start` | 从零力矩进入默认姿态 | 上升沿触发 |
| G1 `A` | 从默认姿态进入策略 | 上升沿触发 |
| G1 `select/stop` | 结束高层任务 | 随后发送一次 damping |
| PICO 左、右 primary 同时按下 | terrain ↔ teleop | 组合上升沿；必须先松开才能再次切换 |

XRobotToolkit 当前把左手 primary 解析为 `left_key_one`（PICO 左 X），把右手 primary 解析为
`right_key_one`（PICO 右 A）。配置按 SDK 键名而不是印刷字母工作；如设备固件上报不同，应先查看
`serve_xrobot_teleop.py` 控制帧，再修改两个任务 YAML 的 `policy_switch.combo_buttons`。

键盘虚拟摇杆和实机摇杆死区均为 `0.1`。首次实机运行如需进一步限制速度，应先修改
`sim2real/config/g1/tap-terrain-real.yaml` 中的 `max_vx` 和 `max_wz`，再启动任务。

## 8. 失效行为与停止顺序

已实现的高层行为：

- 首帧深度到达前：保持训练默认姿态并持续等待；sim2sim 不发送控制使能，因此临时基座稳定器
  不会提前释放；
- 首帧有效深度到达后：构造训练一致的零填充历史，立即执行 ONNX，并将首个 action 与
  `enable=1` 在同一命令中发送；
- 深度曾经就绪后又丢失、超时超过 0.25 秒、非有限或无效比例超过 60%：退出策略并发送一次 damping；
- teleop 模式不消费深度；切回 terrain 前先检查最新深度，深度无效时拒绝切换并继续 teleop；
- PICO 组合键只接受 0.25 秒内的新鲜控制帧，按住组合键不会反复切换；
- 每次切换会保存当前 policy 的记录分段，并为新 policy 创建独立记录，避免 7484/8809 维观测
  混入同一个 NPZ；
- bridge 状态超过 1 秒未到达：退出策略并发送一次 damping；
- 收到 `stop`：结束任务并发送一次 damping；
- 正常退出、`Ctrl+C` 或捕获到异常：发送一次 damping，并保存已有记录；
- C++ bridge 正常关闭时：bridge 自身再发送一次 damping；
- `[MotorDiag]` 只报告和记录异常，不会自动修改命令或进入 damping。

高层软件停止依赖 UDP、Python 进程和 bridge 正常工作，不等价于安全认证的硬件急停。机器人失稳、
通信异常或软件无响应时，应优先使用 G1 实体停止装置。

正常停止顺序：

1. 摇杆回中，等待机器人停止移动；
2. 按 G1 遥控器 `select/stop`；
3. 确认策略进程已经发送 damping 并退出；
4. `Ctrl+C` 停止 G1 bridge；
5. 停止 D435i 发送端。

## 9. 快速排障

### 一直显示 `waiting for depth: depth_missing`

- 检查 G1 相机进程是否仍在运行；
- 检查策略机到 `${ROBOT_IP}:28811` 的网络路由和防火墙；
- 确认启动命令传入了正确的 `--depth-connect`；
- 确认发送端协议为 `d435i-raw-z16-v1`，不是已经预处理的 float 深度。

### 提示 `depth_stale` 或 `depth_invalid_fraction`

- `depth_stale`：检查 Wi-Fi/有线网络抖动、相机帧率和 G1 负载；
- 无效比例过高：检查保护膜、USB 3、镜头遮挡、强光、反光/透明物体和过近障碍物；
- 不要通过放宽 0.25 秒超时来掩盖持续断流，应先解决发送端或网络问题。

### 提示缺少 `tau_latest`

当前策略的 estimator 历史必须包含最新关节估计力矩。重新编译当前仓库的
`g1_sim2real`，并确认运行的不是旧 build 目录中的二进制。

### ONNX 或 JSON 找不到

分别确认 `0818_terrain` 与 `0819_teleop` 的 ONNX/JSON 配对存在，并检查
`tap-terrain-policy.yaml`、`tap-teleop-policy.yaml` 的相对路径。每个 ONNX 必须和自己的同名 JSON
放在同一目录，不能只复制 `.onnx`。

### PICO 组合键没有切换 policy

- 确认 XRoboToolkit PC Service、PICO 和 `scripts/run_pico_server.sh` 均在运行；
- 确认策略日志显示 `VRMotionSource` 已连接 `28701/28702/28703`；
- 必须先完全松开两个 primary，再同时按下；按住不会重复触发；
- 当前 SDK 键名是 `left_key_one + right_key_one`，对应标准 PICO 左 X + 右 A；
- 若日志显示 `PICO control age ... exceeds 0.250s`，先解决控制帧断流，不要放宽超时；
- 切回 terrain 被拒绝时，按日志中的 `depth_missing/depth_stale` 原因恢复 D435i。

### sim2sim 深度全为远值或策略不响应坡道

- 必须使用 `bridge_tap_terrain.yaml`，不能替换成旧 locomani bridge；
- 深度相机必须使用 `tap-terrain.yaml`，正常输出为 `(1,18,32)`；
- 确认场景中存在 `tap_terrain_depth_camera` site；
- 不要启动旧的 `1×36×64` 深度点云调试叠加层。

### sim2sim 按 P（或 sim2real 按 A）后仍然立即倒地

- 完全退出并重新启动 MuJoCo bridge、深度相机和 TAP 策略，避免旧 Python 进程继续运行修复前代码；
- 确认启动的是 `config/g1/bridge_tap_terrain.yaml`，且日志显示
  `startup.default_pose_mode=sim2real_pd`；
- 确认 `tap-terrain.yaml` 中 `history_warmup_steps: 0`；
- 当前 TAP 仿真使用 `P` 进入 policy，不再使用 `A`；`A` 已用于左移；
- 正常时按 P 后不会等待 50 帧；策略日志出现 `depth ready` 后应立即开始动作输出；
- 如果仍倒地，保留三个终端从按 `R` 前到倒地后的完整日志，并保留对应策略 NPZ 记录，以便核对
  首帧姿态、深度、action、关节力矩和通信间隔。

### WASD/QE 没有产生运动

- 确认终端 1 使用 `bridge_tap_terrain.yaml`，启动时应打印完整 keyboard 映射；
- 确认 `tap-terrain.yaml` 的 `velocity_command.source` 为 `sticks`；
- 必须先按 `R`、等待默认姿态稳定，再按 `P` 进入 policy；运动键在 policy 接管后才会驱动机器人；
- 确保终端 1 保持输入焦点，并检查按键时是否打印 `Key pressed: w` 等日志；
- 不要使用 `--velocity` 启动策略进程，否则该参数会显式切换为固定速度并覆盖键盘指令。

## 10. 验证边界

已确认：两组 ONNX/JSON 完整、ONNX Runtime 前向正常、7484/8809 维拼接正确、TAP 历史 reset
语义与训练实现一致、训练侧深度预处理数值一致、相机外参转换正确、坡道起点深度有效，71 项
自动化测试全部通过。自动化测试也确认组合键按住只产生一次切换边沿；尚未把双向切换计入下述
物理闭环结果。

已确认的仿真接管结果：模拟按 P 后深度延迟 1 秒时，临时稳定器保持期间 base 姿态约为
`roll=-0.01°、pitch=0°`；有效深度到达后由首个 policy action 原子接管，继续运行 3 秒时 base
高度约为 `0.792 m`，未倒地。该结果使用当前 checkpoint、现有关节映射、PD 参数、相机和坡道
场景得到。

已确认的零速结果：速度命令持续为 `[0,0,0]`、不按 WASD/QE 时，当前 ONNX 在坡道通道起点
闭环运行 3 秒后 base 高度约为 `0.774 m`，水平位置约从 `[-1.000,-10.000] m` 变为
`[-0.980,-9.994] m`，未倒地。该短时结果确认了零速启动可用，但不等价于长时间原地保持或真机
静态稳定性验证。

尚未确认：真实 D435i 的安装误差、视场角误差、网络时延分布、真机力矩估计分布，以及该
checkpoint 在真实坡道上的闭环稳定性。完成吊装低速测试前，不应将本任务标记为真机部署已验证。

## 11. G1 全机载部署（实验性）

terrain 模式本身不使用 PICO，可以把 D435i 发送端、双 policy 进程和 G1 bridge 全部放到 G1
机载计算机。此时深度 ZMQ、PICO ZMQ（启用 teleop 时）和策略/bridge UDP 都使用
`127.0.0.1`；实时计算闭环不需要外部主机，但 teleop 模式仍需要 PICO 和机载 XRoboToolkit
Service。当前已确认的是组件寻址和两个 ONNX 的单次前向；尚未确认 D435i、XR 重定向、双模型
常驻和 DDS bridge 并发时能在目标 G1 上持续满足 50 Hz，也尚未完成全机载真机闭环。

### 11.1 首次准备

1. 按第 5.3 节在 G1 上准备独立 `g1-camera-stream` Python 3.12 环境和源码构建的
   `pyrealsense2`；只连接本地外接 D435i，不接宇树内置相机链路。
2. 把完整仓库复制到 G1，在主仓库 `sim2real/` 中执行 `uv sync`。
3. 在 G1 上重新编译 bridge：

   ```bash
   cd <repo>/g1_sim2real
   G1_BRIDGE_BUILD_DIR=build_tap_onboard bash scripts/build.sh
   ```

4. 按第 5.3 节把本地 D435i 实际序列号写入 `/home/unitree/g1-camera-stream/camera.yaml`，并确认
   `640×360@30`、`89.041605°×57.9°` 和 pelvis 安装外参与两个 TAP task YAML 一致。
5. 确认两个 policy 配置保持默认的仓库内路径：

   ```yaml
   # tap-terrain-policy.yaml
   policy_path: "../../../ckpts/0818_terrain/model_18000_play_full.onnx"

   # tap-teleop-policy.yaml
   policy_path: "../../../ckpts/0819_teleop/model_1000_play_full.onnx"
   ```

6. 使用本文第 2 节 SHA-256 校验两组 ONNX 和 JSON，不能只复制 ONNX。需要 teleop 时，还要按
   [机载部署总说明](onboard-deployment_zh.md) 安装 ARM64/headless XR 服务和 Python binding。

### 11.2 机载启动顺序

终端 1，用相机分支的本地配置启动外接 D435i 发送端：

```bash
cd /home/unitree/g1-camera-stream
taskset -c 0-1 .venv/bin/python depth_camera_sender.py --config camera.yaml
```

可选：需要切换到 teleop 时，在启动策略前启动机载 XRoboToolkit PC Service 和 PICO 重定向；
PICO 连接 G1 无线 IP：

```bash
cd /opt/apps/roboticsservice
bash runService.sh

cd <repo>/sim2real
bash scripts/run_pico_server.sh
```

终端 2，启动机载 bridge；`eth0` 是 DDS 网卡名，不是 IP：

```bash
cd <repo>/g1_sim2real
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_tap_onboard \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
taskset -c 2-3 bash scripts/run_bridge.sh
```

终端 3，策略直接使用 `tap-terrain-real.yaml` 中的本机深度地址
`tcp://127.0.0.1:28811`：

```bash
cd <repo>/sim2real
taskset -c 4-7 uv run src/tap_terrain.py --target real
```

首次排查可追加 `--no-record` 降低 I/O；完成周期稳定性验证后，正式吊装测试应恢复记录。不要再
传入外部 `${ROBOT_IP}:28811`。只运行 terrain 时不要求 XR/PICO；需要组合键切换时，上述 XR
服务与 `run_pico_server.sh` 必须保持运行。

### 11.3 机载验收与回退条件

进入控制前必须同时确认：日志中的 D435i 序列号等于本地配置、FOV 与 TAP 配置一致、稳定输出
`640x360@30`、策略显示 `depth ready`、bridge 持续提供 `tau_latest`、控制周期满足 50 Hz。启用
teleop 时还要确认 PICO 控制帧持续更新、组合键只
触发一次、切换日志和 1 秒混合正常。持续出现 `depth_stale`、PICO stale、bridge state timeout、
策略周期抖动或机载温度/负载异常时，应停止任务并退回第 6 节外部策略机方案；不要放宽 0.25 秒
超时来掩盖性能不足。全机载首次实机仍须使用吊架、低速命令和 G1 实体停止装置。
