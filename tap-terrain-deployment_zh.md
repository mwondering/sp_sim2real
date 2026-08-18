# TAP—terrain sim2sim / sim2real 部署说明

最后更新：2026-08-18。

本文描述 `TAP—terrain` 任务在 Unitree G1 上的仿真和真机启动方式。该任务直接运行完整的
`model_18000_play_full.onnx`，外部适配层只负责构造机器人历史、速度指令和深度图，不重复执行
ONNX 内部的 estimator、深度编码器或 decoder。

当前状态：sim2sim 的模型加载、深度链路、相机外参、ONNX 前向和 A 键原子接管已经通过自动化
测试与同模型物理闭环验证；sim2real 接口已经实现，但该 checkpoint 尚未完成 D435i/G1 实机
闭环验证。首次上机必须使用可靠吊架或保护绳，并确保操作者可立即触发实体停止装置。

## 1. 部署合同

| 项目 | 值 |
| --- | --- |
| 任务名 | `TAP—terrain` |
| 策略输入 | `spv5_2_terrain_observation`，`float32[1,7484]` |
| 策略输出 | `action`，`float32[1,29]` |
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
```

checkpoint 当前位于仓库外的同级目录：

```text
/home/lenovo/workspace/UNICTL/terrain_bfm_ckpts/model_18000_play_full.onnx
/home/lenovo/workspace/UNICTL/terrain_bfm_ckpts/model_18000_play_full.json
```

策略配置使用相对路径访问这两个文件。移动仓库或在另一台策略计算机部署时，必须保持相同目录
关系，或者同步修改 `tap-terrain-policy.yaml` 中的 `policy_path`。

校验文件：

```bash
sha256sum \
  /home/lenovo/workspace/UNICTL/terrain_bfm_ckpts/model_18000_play_full.onnx \
  /home/lenovo/workspace/UNICTL/terrain_bfm_ckpts/model_18000_play_full.json
```

预期 SHA-256：

```text
655c27340a8a254e7e0fed8c4857969a3266823464dfcba09f8c631c11ace25a  model_18000_play_full.onnx
8cfe20694b10f42d210bff4a6f0e5dad1fb3d9bed0287c042d1b3b9955dd2201  model_18000_play_full.json
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

测试应通过 8 项检查，包括 7484 维布局、训练一致的历史初始化、训练侧深度预处理、精确相机
外参、坡道有效深度、真实 ONNX 输入输出、一次完整策略前向，以及 A 键到首个策略动作的使能
时序。

运行完整回归测试：

```bash
uv run python -m unittest discover -s tests -v
```

当前完整测试集为 66 项。

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

### 进入控制

TAP 启动键和运动键均输入到终端 1，而不是策略终端。为避免 `S/A` 与 WASD 冲突，TAP 专用
bridge 将旧启动键重映射为 `R/P/X`。

1. 三个进程均已运行后，在终端 1 按 `r`。
2. 等待机器人进入训练默认姿态，并确认 grounded PD hold 已稳定。
3. 确认机器人初始速度指令为 `[0,0,0]`，然后在终端 1 按 `p`。
4. P 键只选择 TAP 任务；有效深度未到达时，sim2sim 继续保留临时基座稳定器。
5. 策略进程打印 `depth ready` 后，首个策略动作与控制使能在同一帧发送，同时释放稳定器。
6. 机器人应立即由策略接管并保持零速度；按住 WASD/QE 后才产生运动命令，松开后相应轴回零。
7. 停止时在终端 1 按 `x`，让高层任务发送 damping 并退出。
8. 分别在终端 1、终端 2 按 `Ctrl+C`，结束 MuJoCo 和深度相机进程。

## 5. sim2real 首次准备

### 5.1 物理安全与相机安装

首次真机运行前必须满足：

- G1 使用可靠吊架或保护绳，脚下和前方无人、无松动物体；
- 一名操作者始终握住 G1 实体遥控器，并熟悉硬件停止操作；
- D435i 固定在 pelvis，位置约为 `[0.12546,0.0197,-0.05952] m`；
- 相机中心射线向前下方倾斜 60°，镜头无遮挡，USB 3 连接稳定；
- 相机输出为 `640×360@30 Hz` 原始 Z16 深度；
- 已经使用低速命令完成 sim2sim 验证。

仿真使用了训练侧精确四元数；真机支架的位置、角度和 D435i 视场角仍需现场测量确认。安装误差
会直接造成深度分布偏移，不能仅凭“能收到图像”判断合同一致。

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

真机深度发送端使用独立的 `g1-camera-stream` 分支，在 G1 机载计算机发布：

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

## 6. sim2real 启动

真机最少使用三个终端：D435i 发送端、G1 bridge、策略进程。不需要启动 PICO 或
`serve_xrobot_teleop.py`。

### 终端 1：D435i 原始深度发送端

在策略计算机设置 G1 机载计算机地址：

```bash
export ROBOT_IP="192.168.123.164"
export ROBOT_SSH="unitree@${ROBOT_IP}"
export G1_CAMERA_ROOT="/home/unitree/g1-camera-stream"

ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py"
```

多相机环境必须显式指定 D435i 序列号：

```bash
ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py \
   --serial-number <D435I_SERIAL>"
```

正常日志应包含 `640x360@30`、`raw-z16`、实际设备序列号和 `depth_scale`。

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

真机使用 G1 实体遥控器，不使用 PICO。策略进入前保持所有摇杆回中。

1. 确认吊装保护、相机日志、LowState 和 bridge 日志正常。
2. 启动策略进程，确认 ONNX 加载成功且显示 `Action mapping: 29/29 mapped`。
3. 按 G1 遥控器 `start`，机器人在约 2 秒内进入训练默认姿态。
4. 等待机器人完全稳定，再按 G1 遥控器 `A` 进入任务。
5. 保持摇杆回中；`depth ready` 出现后，策略立即接管。
6. 先用很小的左摇杆 Y 正值验证低速直行，再逐步增加；不要首次运行就给满量程。
7. 发现姿态、深度或动作异常时，立即使用 G1 实体停止装置。

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

键盘虚拟摇杆和实机摇杆死区均为 `0.1`。首次实机运行如需进一步限制速度，应先修改
`sim2real/config/g1/tap-terrain-real.yaml` 中的 `max_vx` 和 `max_wz`，再启动任务。

## 8. 失效行为与停止顺序

已实现的高层行为：

- 首帧深度到达前：保持训练默认姿态并持续等待；sim2sim 不发送控制使能，因此临时基座稳定器
  不会提前释放；
- 首帧有效深度到达后：构造训练一致的零填充历史，立即执行 ONNX，并将首个 action 与
  `enable=1` 在同一命令中发送；
- 深度曾经就绪后又丢失、超时超过 0.25 秒、非有限或无效比例超过 60%：退出策略并发送一次 damping；
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

确认两个文件都存在，并检查 `tap-terrain-policy.yaml` 的相对路径。ONNX 和 JSON 必须放在同一目录，
不能只复制 `.onnx`。

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

已确认：ONNX/JSON 完整、ONNX Runtime 前向正常、7484 维拼接正确、TAP 历史 reset 语义与训练
实现一致、训练侧深度预处理数值一致、相机外参转换正确、坡道起点深度有效，66 项自动化测试
全部通过。

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
