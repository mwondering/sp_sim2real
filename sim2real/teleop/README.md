# PICO / XRoboToolkit 与 G1 Retarget

本目录包含外部主机侧的完整 reference 链路：

- `serve_xrobot_teleop.py`：实时读取 PICO、retarget、ZMQ 发送和 Viser。
- `record_xrobot_motion.py`：录制未 retarget 的 PICO 原始流。
- `replay_xrobot_motion.py`：回放原始流并重新 retarget，可同时作为 ZMQ server。
- `resample_xrobot_raw_npz.py`：按时间戳重采样原始录制。
- `convert_motion_to_isaaclab.py`：把旧 robot-motion NPZ 批量转换为
  IsaacLab/Sonic NPZ。
- `serve_motion_reference.py`：直接播放已有 G1 robot motion。
- `motion_select.py`：在 motion server 运行期间按编号或名称切换下一条 motion。
- `retarget/`：MuJoCo/Mink G1 retarget 与 Viser。
- `utils/`：XR 数据解析、共享缓冲区和 SDK 加载。

## 依赖组成

实时 PICO 需要两个独立组件：

1. XRoboToolkit PC Service，默认由 `/opt/apps/roboticsservice/runService.sh` 启动；
2. `xrobotoolkit_sdk` Python binding，安装到本仓库的 `sim2real/.venv`。

先安装系统构建依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  libprotobuf-dev protobuf-compiler \
  libgrpc++-dev libgrpc-dev protobuf-compiler-grpc \
  libgl1 libegl1 libxrender1 libxext6
```

然后安装 Python 环境和 binding：

```bash
cd sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

安装脚本会把 XRoboToolkit 源码克隆到被 Git 忽略的 `teleop/deps/`，从源码构建 SDK，随后
安装 Python binding。

## 验证

```bash
cd sim2real
uv run python -c 'import xrobotoolkit_sdk, mujoco, mink, zmq; print("imports: OK")'
test -x /opt/apps/roboticsservice/runService.sh
```

连接 PICO 并启动 PC Service 后，可以直接运行：

```bash
cd sim2real
uv run python teleop/serve_xrobot_teleop.py \
  --robot g1 \
  --config config/g1/retarget/teleop.yaml \
  --req-bind-addr tcp://0.0.0.0:28701 \
  --rep-bind-addr tcp://0.0.0.0:28702 \
  --ctrl-bind-addr tcp://0.0.0.0:28703 \
  --viewer-host 0.0.0.0 --viewer-port 8080
```

正常使用建议执行上层 `scripts/launch_pico.sh`，它会同时管理 PC Service 和 reference
server。策略推理和机器人控制不在本分支中运行。

motion 模式会自动扫描仓库根目录的 `motion`，不需要在启动时指定文件；launcher 会用
`28704` 本地端口启动 `motion_select.py`。机载 deploy 启动后只需按一次 `A` 进入策略控制。
此后每次选择都会在策略就绪后自动执行；motion 结束后，策略自动过渡到默认 reference 并
保持控制，等待下一次选择。

## Motion NPZ 格式与转换

播放器原生支持两种 G1 NPZ：

- IsaacLab/Sonic（推荐）：`joint_pos`、`body_pos_w`、`body_quat_w`；其中
  `joint_pos` 使用内置的 G1 IsaacLab 29 关节顺序，`body_quat_w` 为 wxyz，默认 body `0`
  是 pelvis/root。`joint_vel`、`body_lin_vel_w` 和 `body_ang_vel_w` 可以存在，但播放器不依赖
  这些速度字段。
- Robot motion（兼容）：`dof_pos`、`root_pos`、`root_rot` 和 `joint_names`；其中
  `root_rot` 为 xyzw，关节通过内嵌名称重排。

把仓库已有的 `config/g1/motions` 批量转换为第一种格式：

```bash
cd sim2real
uv run python teleop/convert_motion_to_isaaclab.py
```

默认直接平铺输出到仓库根目录的 `motion`；转换前会检查重名，已有输出也不会被覆盖。明确
需要重新生成时添加 `--overwrite`。也可以指定其他输入和输出：

```bash
uv run python teleop/convert_motion_to_isaaclab.py /path/to/input \
  --output-root /path/to/output
```

默认播放仓库根目录 `motion` 下的 IsaacLab/Sonic 文件：

```bash
cd sim2real
bash scripts/launch_pico.sh --source motion
```

播放器也会递归扫描用户后来添加的子目录。
