# PICO 分支启动说明

## 分支边界

本分支只运行在外部主机，保留 PICO 通信、G1 retarget、motion 数据与播放、ZMQ reference
发送和 Viser 可视化。它不包含也不加载任何 checkpoint。

## 终端：外部主机

```bash
git switch pico
cd sim2real

SERVER_IP=192.168.31.20 \
REFERENCE_BIND_IP=0.0.0.0 \
VR_REQ_PORT=28701 \
VR_POSE_PORT=28702 \
VR_CTRL_PORT=28703 \
VIEWER_BIND_IP=0.0.0.0 \
VIEWER_PORT=8080 \
XR_SERVICE_DIR=/opt/apps/roboticsservice \
bash scripts/launch_pico.sh --source pico
```

G1 的 `deploy` 分支必须把 `REFERENCE_HOST` 配成这里的 `SERVER_IP`，并使用完全相同的三个
`VR_*_PORT`。`SERVER_IP` 只用于显示浏览器地址，真正监听地址由 `REFERENCE_BIND_IP` 和
`VIEWER_BIND_IP` 控制。

## Motion 模式

```bash
cd sim2real
MOTION_FILE="$PWD/config/g1/motions/omni_extreme/omni_extreme_1.npz" \
bash scripts/launch_pico.sh --source motion --motion-file "$MOTION_FILE"
```

原始 PICO 录制回放：

```bash
cd sim2real
bash scripts/launch_pico.sh --source raw-replay \
  --motion-file /path/to/xrobot_raw.npz
```

## tmux 操作

```bash
tmux attach -t g1_spv5_pico
bash scripts/launch_pico.sh --stop
```

窗口职责：

- `xr-service`：实时模式运行 XRoboToolkit PC Service；motion 模式不使用。
- `reference`：retarget/reference server 与 Viser。

## 网络消息

- `VR_REQ_PORT`：deploy → pico，请求下一帧。
- `VR_POSE_PORT`：pico → deploy，发送 `g1-reference-v1` header 与 36 维 `float32` qpos。
- `VR_CTRL_PORT`：pico → deploy，发送手柄按键、摇杆和时间戳 JSON。
- `VIEWER_PORT`：浏览器可视化，默认 `8080`。
