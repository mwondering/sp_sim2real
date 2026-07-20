# Motion Tracking Sim2Real 中文使用说明

[English README](README.md)

本仓库提供运动跟踪策略的 sim2sim 与 sim2real 部署运行时，当前支持：

- G1 sim2sim
- L7 sim2sim
- G1 sim2real
- G1 SP_Tracking WBTeleop actor
- G1 SP_Tracking SPV5-1 actor
- Legacy NPZ 与 IsaacLab/Sonic NPZ motion
- UDP motion 播放与 XR/PICO 实时遥操作

本仓库不包含训练代码和数据集生成代码，也不包含 L7 实机底层桥接程序。

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

## 快速开始：SPV5-1 + IsaacLab/Sonic motion

仓库中的 `config/g1/tracking_spv5_1.yaml` 已经直接配置好当前 SPV5-1 checkpoint 和三条内置 motion，无需复制配置文件。关键配置如下：

```yaml
policy_path: "/home/lenovo/workspace/UNICTL/SP_Tracking/logs/ckpts/0720/policy.onnx"
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
  - name: "jumps1_subject1"
    path: "../../../motion/jumps1_subject1.npz"
    start: 0
    end: -1
  - name: "flip_360_001__A304"
    path: "../../../motion/flip_360_001__A304.npz"
    start: 0
    end: -1
  - name: "dance1_subject2_0_3945"
    path: "../../../motion/dance1_subject2_0_3945.npz"
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
3. 在 motion 选择器中输入 `flip_360_001__A304`，或输入对应序号。
4. motion 执行结束后，如需切换动作，先选择 `default`，再选择下一个非默认动作。
5. 在 MuJoCo 窗口按 `x` 停止控制。

上述原始文件已直接完成回归测试：motion 被识别为 358 帧 IsaacLab/Sonic 数据，ONNX 输入/输出维度为 8199/29，运行时不需要转换 NPZ，也不需要 `/tmp` metadata 兼容配置。

## Actor 与 checkpoint 配置

### 内置策略配置

| 配置文件 | 用途 | ONNX 输入宽度 |
| --- | --- | ---: |
| `tracking.yaml` | 默认 G1/L7 PMG | 由模型 metadata 决定 |
| `tracking_compliance.yaml` | G1 compliance policy | 由模型 metadata 决定 |
| `tracking_wbteleop.yaml` | SP_Tracking WBTeleop actor | 886 |
| `tracking_spv5_1.yaml` | SP_Tracking SPV5-1 actor | 8199 |

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
```

也可以直接在 YAML 中把 `policy_path` 设置为绝对路径。运行时会检查 actor profile 与 ONNX 输入宽度是否匹配。

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

SPV5-1 observation 包含关节力矩历史，因此 sim2sim 或 G1 底层桥接状态必须提供 `tau`。仓库当前版本已经支持；旧版桥接程序会被明确拒绝，不会静默使用全零力矩。

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
uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_1.yaml
```

UDP 模式在另一个终端启动相同 tracking 配置的 motion 选择器。VR 模式应先启动遥操作桥。

### G1 机载运行

机载部署需要把以下两个目录复制到 G1 机载计算机：

- `sim2real/`：Python 策略运行时。
- `g1_sim2real/`：C++ 底层桥。

机载网卡通常使用 `eth0`：

```bash
cd <repo>/g1_sim2real
G1_NET=eth0 bash scripts/run_bridge.sh
```

G1 机载 CPU 性能有限，建议把不同进程绑定到不同核心：

```bash
# 终端 1：VR 遥操作桥
cd <repo>/sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1

# 终端 2：底层桥
cd <repo>/g1_sim2real
G1_NET=eth0 taskset -c 2-3 bash scripts/run_bridge.sh

# 终端 3：策略推理
cd <repo>/sim2real
taskset -c 4-7 uv run src/deploy.py --robot g1 --no-record \
  --tracking-config tracking_spv5_1.yaml
```

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

## 常见问题

### motion 能加载，但动作明显错误

首先检查 `motion_type`。Sonic/IsaacLab 原始数据应使用 `isaaclab`；只有 `joint_pos` 本身已经是 `dataset_joint_names` 顺序时才使用 `mujoco`。不要仅根据数组宽度判断顺序，两个格式都可能是 29 维。

### 报错缺少 `tau`

SPV5-1 需要真实的关节力矩反馈历史。请重新编译并启动当前仓库中的 sim2sim/G1 bridge，不要使用旧版桥接程序。

### 报错 observation 维度不匹配

检查 checkpoint 与 actor profile 是否配套：WBTeleop 的输入宽度为 886，SPV5-1 为 8199。同时确认 `policy.onnx` 与 `policy.json` 来自同一个导出目录。

### motion 选择器看不到新增动作

确认 `motion_select.py` 和 `deploy.py` 使用完全相同的 `--tracking-config`。修改 YAML 后在选择器输入 `r` 重新加载。

### 相对路径找不到文件

`policy_path` 和 motion `path` 的相对路径以当前 tracking YAML 所在目录为基准。跨仓库文件建议使用绝对路径，或复制到 `sim2real/config/<robot>/ckpts` 与 `motions` 目录。
