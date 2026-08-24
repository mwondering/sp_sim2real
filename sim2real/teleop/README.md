# PICO / XRoboToolkit 与 G1 Retarget

本目录包含外部主机侧的完整 reference 链路：

- `serve_xrobot_teleop.py`：实时读取 PICO、retarget、ZMQ 发送和 Viser。
- `record_xrobot_motion.py`：录制未 retarget 的 PICO 原始流。
- `replay_xrobot_motion.py`：回放原始流并重新 retarget，可同时作为 ZMQ server。
- `resample_xrobot_raw_npz.py`：按时间戳重采样原始录制。
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

motion 模式下，launcher 会用 `28704` 本地端口启动 `motion_select.py`。motion 单次播放；
完成后在 G1 遥控器按 `Up` 返回默认位姿，选择下一条 motion 后按 `A` 再次开始。
