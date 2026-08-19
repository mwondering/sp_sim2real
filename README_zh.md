# Motion Tracking Sim2Real 中文使用说明

[English README](README.md)

本仓库提供运动跟踪策略的 sim2sim 与 sim2real 部署运行时，当前支持：

- G1 sim2sim
- L7 sim2sim
- G1 sim2real
- G1 SP_Tracking WBTeleop actor
- G1 SP_Tracking SPV5-1 actor
- G1 SP_Tracking SPV5-2 actor
- Legacy NPZ 与 IsaacLab/Sonic NPZ motion
- UDP motion 播放与 XR/PICO 实时遥操作

本仓库不包含训练代码和数据集生成代码，也不包含 L7 实机底层桥接程序。

Dual-Teacher 的完整启动顺序见 [Dual-Teacher sim2sim / sim2real 部署说明](dual-teacher-deployment_zh.md)。
所有任务的机载适用边界、公共安装和通信拓扑见 [G1 机载部署总说明](onboard-deployment_zh.md)。

## 目录结构

```text
sim2real/
  src/                 Python 策略运行时、sim2sim 和 motion 选择器
  config/g1/           G1 控制、策略、MuJoCo、motion 与重定向配置
  config/l7/           L7 控制、策略、MuJoCo、motion 与重定向配置
  teleop/              XR/PICO 实时遥操作桥接程序

motion/                 仓库自带的 IsaacLab/Sonic NPZ motion

g1_sim2real/
  src/                 G1 C++ UDP/DDS 底层桥接程序
  config/              G1 桥接配置
  scripts/             编译与启动脚本
  third_party/         C++ 第三方依赖
```

## 环境安装

Python 运行时要求 Python 3.10。推荐使用 `uv` 创建和管理环境：

```bash
cd <repo>/sim2real
uv sync
```

除非单独说明，后续 Python 命令都应在 `<repo>/sim2real` 下执行。支持的机器人参数为 `g1` 和 `l7`。

## SPV5-1 + IsaacLab/Sonic motion（需要补 checkpoint）

仓库中的 `config/g1/tracking_spv5_1.yaml` 已提供 SPV5-1 的观测、关节、PD、motion 和 VR
通信合同，但当前 checkout **不包含** `config/g1/ckpts/G1_SPV5_1/policy.onnx` 与同名 JSON
sidecar。必须先从同一个导出目录复制完整 checkpoint，才能执行本节命令。关键配置为：

```yaml
policy_path: "ckpts/G1_SPV5_1/policy.onnx"
actor_profile: "spv5_1"
use_policy_metadata: true

motion_type: "isaaclab"
root_body_index: 0

motion_source:
  type: "udp"
  udp:
    enable: true
    host: "127.0.0.1"
    port: 28562

motions:
  - name: "walk1_subject1"
    path: "motions/walk1_subject1.npz"
    start: 0
    end: -1
```

旧版 SPV5-1 导出的 `policy.json` 可以缺少关节、动作缩放和 PD 参数，但仍需保留与 `policy.onnx` 同名且同目录的 sidecar；仓库配置只补齐缺失的部署字段，不覆盖 sidecar 已提供的字段。如果 checkpoint 位置发生变化，只需修改原始 `tracking_spv5_1.yaml` 中的 `policy_path`。

打开三个终端。

终端 1，启动 MuJoCo sim2sim：

```bash
cd <repo>/sim2real
uv run src/sim2sim.py --robot g1
```

G1 的 `bridge.yaml` 默认启用 `startup.default_pose_mode: sim2real_pd`。按 `s` 后，sim2sim 先跟随 deploy 的两秒插值到策略元数据中的默认关节角；目标稳定后，机器人落地并用 deploy 下发的 Kp、Kd 做动力学 PD 保持。等待终端打印 `grounded PD hold is active` 后再按 `a`，此时临时的基座稳定力会被撤掉，策略接管时保留已有的 q、dq、torque 和接触状态。基座稳定仅用于避免无策略等待阶段直接摔倒，不作用于策略阶段。若需要复现旧版零速度、零扭矩的悬空瞬移交接，可临时改为 `teleport`。

终端 2，启动 SPV5-1 策略控制器：

```bash
cd <repo>/sim2real
uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_1.yaml
```

终端 3，启动 motion 选择器：

```bash
cd <repo>/sim2real
uv run src/motion_select.py --robot g1 --tracking-config tracking_spv5_1.yaml
```

启动后的操作顺序如下：

1. 保持 MuJoCo 窗口获得键盘焦点，按 `s`，机器人从零力矩状态移动到默认姿态。
2. 等待默认姿态过渡完成，按 `a` 进入 tracking policy。
3. 在 motion 选择器中输入仓库内可解析的 `walk1_subject1`，或输入对应序号；使用其他
   IsaacLab/Sonic motion 前先确认其路径在当前主机存在。
4. motion 执行结束后，如需切换动作，先选择 `default`，再选择下一个非默认动作。
5. 在 MuJoCo 窗口按 `x` 停止控制。

仓库自动化测试覆盖 SPV5-1 的 8199 维观测合同、metadata fallback、IsaacLab/Sonic 重排和
`tau` 语义；这不代表当前缺失的外部 checkpoint 已在本 checkout 中完成前向或真机验证。

## Actor 与 checkpoint 配置

### 内置策略配置

| 配置文件 | 用途 | ONNX 输入宽度 |
| --- | --- | ---: |
| `tracking.yaml` | 默认 G1/L7 PMG | 由模型 metadata 决定 |
| `tracking_compliance.yaml` | G1 compliance policy | 由模型 metadata 决定 |
| `tracking_wbteleop.yaml` | SP_Tracking WBTeleop actor | 886 |
| `tracking_spv5_1.yaml` | SP_Tracking SPV5-1 actor | 8199 |
| `tracking_spv5_2.yaml` | SP_Tracking SPV5-2 actor | 8199 |

SP_Tracking 导出文件应来自同一个 checkpoint。典型目录包含：

```text
policy.onnx
policy.json
policy.onnx.data    # 仅外部权重格式的 ONNX 需要
```

可以把文件复制到以下目录并使用配置中的相对路径：

```text
sim2real/config/g1/ckpts/G1_WBTeleop/
sim2real/config/g1/ckpts/G1_SPV5_1/
sim2real/config/g1/ckpts/G1_SPV5_2/
```

也可以直接在 YAML 中把 `policy_path` 设置为绝对路径。运行时会检查 actor profile 与 ONNX 输入宽度是否匹配。

当前 checkout 已包含默认 PMG、compliance 和 SPV5-2 所引用的模型；Dual-Teacher、MJLab
locomani 模型也在各自任务目录中。`tracking_wbteleop.yaml` 和 `tracking_spv5_1.yaml` 指向的
专用 checkpoint 目录尚未随仓库提供，复制到其他主机或机载计算机时必须单独补齐。

### 旧版 SPV5-1 metadata

部分旧版 SP_Tracking exporter 生成的 `policy.json` 只有网络输入输出和 body metadata，缺少以下部署字段：

- `joint_names`
- `action_scale`
- `default_joint_pos`
- `joint_stiffness`
- `joint_damping`

`tracking_spv5_1.yaml` 已永久提供这些字段的 fallback。合并规则为：

1. 先读取 checkpoint 同目录下的 `policy.json`。
2. 只用 YAML fallback 补齐缺失字段。
3. `policy.json` 已存在的同名字段始终优先。

SPV5-1 和 SPV5-2 observation 都包含关节力矩历史，但训练语义不同：SPV5-1 使用控制周期平均值 `tau`，SPV5-2 使用最新样本 `tau_latest`。当前 sim2sim 和 G1 bridge 会同时发送两者，运行时按 actor profile 自动选择；旧版 bridge 会被明确拒绝。

## Motion NPZ 格式

同一份 `motions` 列表可以同时加载两类格式。

### Legacy 格式

必须包含：

- `dof_pos`：形状 `[T, J]`
- `root_pos`：形状 `[T, 3]`
- `root_rot`：形状 `[T, 4]`，四元数顺序为 xyzw
- `joint_names`：长度为 `J` 的关节名列表

Legacy 数据使用 NPZ 内嵌的 `joint_names` 完成重排，不受全局 `motion_type: isaaclab` 影响。

### IsaacLab/Sonic 格式

必须包含：

- `joint_pos`：形状 `[T, J]`
- `body_pos_w`：形状 `[T, B, 3]`
- `body_quat_w`：形状 `[T, B, 4]`，四元数顺序为 wxyz

推荐配置：

```yaml
motion_type: "isaaclab"
root_body_index: 0
motions:
  - name: "example"
    path: "/absolute/path/to/example.npz"
    start: 0
    end: -1
```

运行时会按照内置的 G1 IsaacLab 29 关节顺序读取 `joint_pos`，再重排到当前 actor 的关节顺序。默认把 body `0` 作为 pelvis/root；如果数据集的 root body 不是第 0 个，需要修改 `root_body_index`。

配置含义：

- `motion_type: isaaclab`：无内嵌关节名时按 IsaacLab G1 顺序解释。
- `motion_type: mujoco`：同样使用 `joint_pos/body_pos_w/body_quat_w` 字段，但 `joint_pos` 已经是 `dataset_joint_names` 指定的 MuJoCo 顺序。
- `motion_type: auto`：对 Sonic/IsaacLab 字段默认采用 IsaacLab 顺序。
- `start`：起始帧，包含该帧。
- `end`：结束帧，不包含该帧；`-1` 表示一直读取到最后一帧，并包含最后一帧。

运行时会检查关节数量、重复/缺失关节、帧数、body 维度、非有限数值和零长度四元数。四元数在进入策略前会归一化。

## 控制状态与按键

策略控制器使用统一状态流程：

1. 零力矩等待。
2. 收到 `start` 后移动到默认姿态。
3. 收到 `A` 后进入 tracking policy。
4. 收到 `stop` 后退出。

| 使用场景 | `start` | `A` | `stop` |
| --- | --- | --- | --- |
| sim2sim 窗口 | 键盘 `s` | 键盘 `a` | 键盘 `x` |
| G1 bridge 终端 | 键盘 `s` | 键盘 `a` | 键盘 `x` |
| G1 遥控器 | `start` | `A` | `select` |

sim2sim 使用键盘控制时，需要保持 MuJoCo 窗口获得焦点。

XR/PICO 还有一层独立的实时流控制：

| XR/PICO 按键 | 作用 |
| --- | --- |
| 右手 `A` | 开始或恢复 VR motion 流 |
| 左手 `X` | 暂停 VR motion 流 |

XR 按键只控制 motion source，不能替代进入策略所需的模拟器/机器人 `start` 和 `A`。

## UDP motion 播放

在 tracking YAML 中启用 UDP：

```yaml
motion_source:
  type: "udp"
```

启动选择器时，必须和 `deploy.py` 使用同一个 `--tracking-config`：

```bash
uv run src/motion_select.py --robot g1 --tracking-config tracking_spv5_1.yaml
```

交互命令：

- 输入 motion 序号或名称：发送对应 motion。
- `list`：显示可用 motion。
- 直接回车：重新发送上一次选择。
- `r`：重新加载 YAML。
- `q`：退出。

策略正在执行时，建议先发送 `default`，再切换到另一个非默认 motion，使参考轨迹过渡保持连续。

## VR 实时 motion

在 tracking YAML 中启用 VR：

```yaml
motion_source:
  type: "vr"
```

先按照 `sim2real/teleop/README.md` 完成 XR/PICO 和组件映射配置，然后启动遥操作桥：

```bash
cd <repo>/sim2real/teleop
uv run python serve_xrobot_teleop.py --robot g1
```

L7 使用 `--robot l7`。推荐顺序如下：

1. 启动 XRoboToolkit PC 服务。
2. 在 PICO 客户端完成 tracker/controller 标定。
3. 将 PICO 客户端连接到 PC 服务。
4. 启动 `serve_xrobot_teleop.py`。
5. 启动 sim2sim 或 G1 sim2real。
6. 使用模拟器/机器人 `start` 和 `A` 进入策略。
7. 按右手 XR `A` 开始实时 motion。
8. 按左手 XR `X` 暂停实时 motion。

启动遥操作时保持稳定站姿，以便重定向模块估计 root 高度。

## 通用 G1/L7 sim2sim

以下命令以 G1 为例；L7 将所有命令中的 `g1` 替换为 `l7`。

终端 1：

```bash
cd <repo>/sim2real
uv run src/sim2sim.py --robot g1
```

终端 2，根据 actor 选择一条命令：

```bash
# 默认 PMG
uv run src/deploy.py --robot g1

# G1 compliance
uv run src/deploy.py --robot g1 --tracking-config tracking_compliance.yaml

# SP_Tracking WBTeleop
uv run src/deploy.py --robot g1 --tracking-config tracking_wbteleop.yaml

# SP_Tracking SPV5-1
uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_1.yaml

# SP_Tracking SPV5-2
uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_2.yaml
```

UDP 模式还需在第三个终端启动 `motion_select.py`；VR 模式应在按模拟器 `a` 之前启动 `serve_xrobot_teleop.py`。

## G1 sim2real

G1 实机部署由 `g1_sim2real/` 下的 C++ 底层桥和 `sim2real/` 下的 Python 策略运行时共同完成。

### 编译底层桥

```bash
cd <repo>/g1_sim2real
bash scripts/build.sh
```

### 网络配置

推荐先在本地计算机同时运行 Python 策略和 G1 bridge，并通过网线连接 G1：

1. 用网线连接 G1 与本地计算机。
2. 给本地有线网卡设置 G1 子网内的静态地址，例如 `192.168.123.201/24`。
3. 确认有线网卡名称，例如 `enp3s0` 或 `enx...`。
4. 用该网卡名称设置 `G1_NET`。

终端 1，启动底层桥：

```bash
cd <repo>/g1_sim2real
G1_NET=enp3s0 bash scripts/run_bridge.sh
```

终端 2，启动策略：

```bash
cd <repo>/sim2real
uv run src/deploy.py --robot g1
```

UDP 模式在另一个终端启动相同 tracking 配置的 motion 选择器。VR 模式应先启动遥操作桥。

### G1 机载运行

完整流程和任务状态矩阵见 [G1 机载部署总说明](onboard-deployment_zh.md)。通用 tracking + PICO
机载部署不需要外部策略主机，但仍需要 PICO、trackers、G1 遥控器以及 PICO 到 G1 的无线局域网。
先把以下两个目录和所选 checkpoint 复制到 G1：

- `sim2real/`：Python 策略运行时。
- `g1_sim2real/`：C++ 底层桥。

在 G1 上执行 `uv sync`；使用 PICO 时还需安装 ARM64 XRoboToolkit PC Service，并在 `uv sync`
之后运行 `sim2real/install_xrobottoolkit_sdk.sh`。已有 build 目录可能来自 x86-64，必须在 G1 上
重新编译：

```bash
cd <repo>/g1_sim2real
G1_BRIDGE_BUILD_DIR=build_onboard bash scripts/build.sh
```

VR 模式不要覆盖原始 UDP 配置；复制一个 checkpoint 完整的 tracking YAML，例如默认 PMG：

```bash
cd <repo>/sim2real
cp config/g1/tracking.yaml config/g1/tracking_onboard_vr.yaml
```

将新文件的 `motion_source.type` 改为 `vr`，保留三条 ZMQ 地址为 `127.0.0.1`。机载 CPU
性能有限，建议按下列顺序和核心分配启动：

```bash
# 终端 1：先启动 /opt/apps/roboticsservice/runService.sh，PICO 连接 G1 无线 IP

# 终端 2：仅绑定本机回环的 VR 遥操作桥
cd <repo>/sim2real
taskset -c 1 bash scripts/run_pico_server.sh

# 终端 3：底层桥；eth0 是 DDS 网卡名，不是 IP
cd <repo>/g1_sim2real
G1_NET=eth0 G1_BRIDGE_BUILD_DIR=build_onboard \
  taskset -c 2-3 bash scripts/run_bridge.sh

# 终端 4：策略推理
cd <repo>/sim2real
taskset -c 4-7 uv run src/deploy.py --robot g1 --no-record \
  --tracking-config tracking_onboard_vr.yaml
```

仓库未提供 systemd 开机自启或进程监督配置；不使用外部计算主机不等于已经实现无人值守启动。
Dual-Teacher、upper/lower locomani 和 TAP—terrain 的机载命令与验证边界见各自任务文档。

### 实机安全与启动顺序

警告：任何 motion 上实机前都必须先通过 sim2sim。首次运行新 checkpoint 或大幅动作时，应为机器人提供可靠保护，清空运动范围内的人员和障碍物，并确保操作者可以立即触发急停/阻尼状态。

1. 确认机器人供电、支撑状态和运动空间安全。
2. 启动底层桥，确认网卡参数正确。
3. 启动 `deploy.py`，等待其报告已收到机器人状态。
4. 按遥控器 `start`，使机器人进入默认姿态。
5. 等待默认姿态过渡完成，确认机器人已经安全落地并保持稳定。
6. 按遥控器 `A` 进入 tracking policy。
7. 使用 UDP 选择器或 XR/PICO 输入 motion。
8. 按遥控器 `select` 或配置的停止键退出。

### 电机安全诊断

G1 bridge 默认启用只读诊断。它直接检查每个 `LowState.motor_state` 的 `motorstate` 原始故障码、驱动模式、外壳/绕组温度、电压、位置、速度和估算力矩，同时检查 IMU 角速度。诊断只报警和记录，不会自行修改控制命令或触发阻尼。

默认硬阈值与仓库内 Unitree SDK 的 G1 runtime termination 保持一致：外壳温度 85 °C、绕组温度 120 °C、关节速度 10 rad/s、IMU 角速度 6 rad/s；预警阈值分别为 75 °C、100 °C、8 rad/s、5 rad/s。关节位置和估算力矩还会按照 `sim2real/config/g1/assets/g1.xml` 中的 G1 边界检查。温度/速度阈值、位置预警余量、力矩预警比例和检查开关可在 `g1_sim2real/config/g1_bridge.yaml` 的 `diagnostics` 段调整。

运行时重点查看两处输出：

- bridge 终端的 `[MotorDiag][WARNING]`、`[MotorDiag][CRITICAL]` 和 `[MotorDiag][RECOVERED]` 会给出关节名、原始 `motorstate` 十六进制值、位置、速度、力矩与两路温度；每秒的 `[MotorDiag] status=...` 是当前摘要。
- `deploy.py` 终端会复述诊断状态变化。未使用 `--no-record` 时，策略 NPZ 还会保存 `motor_temperature_*`、`motor_voltage`、`motor_state`、`motor_diagnostic_flags`、`diagnostic_*` 和 `mode_machine` 等字段，便于把进入阻尼前后的电机状态与动作对齐。

`motorstate != 0` 是最直接的底层异常证据，但当前 SDK 未在仓库中提供 G1 故障码到具体原因的完整映射；因此日志保留原始码，不会把未知码猜测成“过热”或“过流”。同样，`LowState` 没有一个能明确说明“因何进入阻尼”的统一字段，诊断结论应以故障码、超限事件和状态切换的时间关系综合判断。

## 常见问题

### motion 能加载，但动作明显错误

首先检查 `motion_type`。Sonic/IsaacLab 原始数据应使用 `isaaclab`；只有 `joint_pos` 本身已经是 `dataset_joint_names` 顺序时才使用 `mujoco`。不要仅根据数组宽度判断顺序，两个格式都可能是 29 维。

### 报错缺少 `tau` 或 `tau_latest`

SPV5-1 需要控制周期平均的 `tau`，SPV5-2 需要最新采样的 `tau_latest`。请重新编译并启动当前仓库中的 sim2sim/G1 bridge，不要使用旧版桥接程序。

### 报错 observation 维度不匹配

检查 checkpoint 与 actor profile 是否配套：WBTeleop 的输入宽度为 886，SPV5-1/SPV5-2 为 8199。两个 SPV5 profile 宽度相同，运行时还会核对 ONNX 输入名。同时确认 `policy.onnx` 与 `policy.json` 来自同一个导出目录。

### motion 选择器看不到新增动作

确认 `motion_select.py` 和 `deploy.py` 使用完全相同的 `--tracking-config`。修改 YAML 后在选择器输入 `r` 重新加载。

### 相对路径找不到文件

`policy_path` 和 motion `path` 的相对路径以当前 tracking YAML 所在目录为基准。跨仓库文件建议使用绝对路径，或复制到 `sim2real/config/<robot>/ckpts` 与 `motions` 目录。
