# G1 D435i 深度发送端

本分支只包含运行在 G1 机载计算机上的 D435i 深度采集与 ZMQ 发送代码，不包含策略、
G1 bridge、ONNX、MuJoCo、PICO 或 XRoboToolkit 依赖。

## 从头配置统一环境

```bash
git clone --branch g1-camera-stream --single-branch \
  https://github.com/mwondering/sp_sim2real.git \
  /home/unitree/g1-camera-stream
cd /home/unitree/g1-camera-stream

curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="${HOME}/.local/bin:${PATH}"

rm -rf .venv
uv python install 3.12
uv venv --python 3.12 .venv

uv pip install \
  --python .venv/bin/python \
  --index-url https://pypi.org/simple \
  -r requirements.txt
```

这里明确使用 Python 3.12，因为官方提供 Linux aarch64 对应的 `pyrealsense2`，而
Python 3.13 没有对应的 aarch64 安装包。

验证统一环境：

```bash
.venv/bin/python -c \
  'import sys, numpy, yaml, zmq; import pyrealsense2 as rs; print(sys.version); print(rs.__file__); print(rs.pipeline)'
```

## 本机启动

默认发布地址为 `tcp://*:28811`，即监听机载计算机所有 IPv4 网卡：

```bash
.venv/bin/python depth_camera_sender.py
```

多相机时指定 D435i 序列号：

```bash
.venv/bin/python depth_camera_sender.py \
  --serial-number <D435i序列号>
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

`Ctrl+C` 会经 SSH 终端传给相机进程并正常关闭 RealSense pipeline。正式使用前应确认
D435i 连接在 USB 3/SuperSpeed 端口，并只允许策略服务器访问 TCP 28811。
