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
SERVER_IP=192.168.31.20 \
REFERENCE_BIND_IP=0.0.0.0 \
VR_REQ_PORT=28701 VR_POSE_PORT=28702 VR_CTRL_PORT=28703 \
MOTION_SELECT_PORT=28704 VIEWER_PORT=8080 \
bash scripts/launch_pico.sh --source motion
```

启动时会自动扫描仓库的 `sim2real/config/g1/motions`，无需手动指定路径。`motion-select`
窗口会列出所有 G1 motion，可输入编号、完整名称或唯一子串选择。每条 motion 只播放一次，
操作顺序如下：

1. 机载 deploy 启动后，在 G1 默认位姿按一次遥控器 `A`，进入并保持策略控制；
2. 在 `motion-select` 选择 motion，机载策略就绪后会自动执行；
3. 播放完成后，策略自动平滑过渡到默认 reference 并继续保持控制；
4. 继续选择下一条 motion，策略会在默认过渡完成后自动执行。

不需要反复按 `A`，也不需要按 `Up`。播放过程中会拒绝替换当前 motion；播放结束后可立即
选择下一条，机载端会先完成默认位姿过渡，再自动启动队列中的 motion。`MOTION_SELECT_PORT`
默认是 `28704`，仅绑定外部主机的 `127.0.0.1`，不需要向 G1 开放。若 motion 不在仓库默认
目录，可通过 `MOTION_ROOT=/path/to/motions` 覆盖扫描路径。

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

- `xr-service`：实时模式运行 XRoboToolkit PC Service。
- `motion-select`：motion 模式下交互选择下一条 G1 motion。
- `reference`：retarget/reference server 与 Viser。

## 网络消息

- `VR_REQ_PORT`：deploy → pico，请求下一帧。
- `VR_POSE_PORT`：pico → deploy，发送 `g1-reference-v1` header 与 36 维 `float32` qpos。
- `VR_CTRL_PORT`：pico → deploy，发送手柄按键、摇杆和时间戳 JSON。
- `MOTION_SELECT_PORT`：外部主机本地 selector → reference server，默认 `28704`。
- `VIEWER_PORT`：浏览器可视化，默认 `8080`。
