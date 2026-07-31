# G1 D435i 深度发送端

本分支只包含运行在 G1 机载计算机上的 D435i 深度采集与 ZMQ 发送代码，不包含策略、
G1 bridge、ONNX、MuJoCo、PICO 或 XRoboToolkit 依赖。

## 安装

```bash
git clone --branch g1-camera-stream --single-branch <仓库地址> g1-camera-stream
cd g1-camera-stream

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

如果当前平台可通过 pip 安装 RealSense Python：

```bash
.venv/bin/python -m pip install pyrealsense2
.venv/bin/python -c \
  'import numpy; import pyrealsense2 as rs; print(rs.__file__, rs.pipeline)'
```

如果机载系统已有另一个能够导入 `numpy` 和 `pyrealsense2` 的 Python，不需要在
`.venv` 中安装 `pyrealsense2`，启动时通过 `--worker-python` 指定该解释器。

## 本机启动

默认发布地址为 `tcp://*:28811`，即监听机载计算机所有 IPv4 网卡：

```bash
.venv/bin/python depth_camera_sender.py
```

指定相机或 RealSense worker：

```bash
.venv/bin/python depth_camera_sender.py \
  --serial-number <D435i序列号> \
  --worker-python /absolute/path/to/realsense/python
```

进程发送 `640×360@30 Hz` 的原始 Z16 深度帧，ZMQ topic 为 `depth`。策略服务器应连接：

```text
tcp://<G1机载计算机的192.168.123.x地址>:28811
```

## 从策略服务器通过 SSH 启动

策略服务器只需要知道 G1 的 SSH 地址和本分支在 G1 上的绝对路径：

```bash
export ROBOT_SSH=unitree@<G1机载计算机的192.168.123.x地址>
export G1_CAMERA_ROOT=/home/unitree/g1-camera-stream

ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && exec .venv/bin/python depth_camera_sender.py"
```

如果使用 worker：

```bash
ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && exec .venv/bin/python depth_camera_sender.py \
   --worker-python /absolute/path/to/realsense/python"
```

`Ctrl+C` 会经 SSH 终端传给相机进程并正常关闭 RealSense pipeline。正式使用前应确认
D435i 连接在 USB 3/SuperSpeed 端口，并只允许策略服务器访问 TCP 28811。
