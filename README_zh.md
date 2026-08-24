# G1 PICO Reference Server

这是配套 `deploy` 分支使用的外部主机分支，只负责：

- 接收 XRoboToolkit/PICO 身体与手柄数据；
- 将人体姿态实时 retarget 为 G1 36 维 qpos；
- 通过三路 ZMQ TCP 向机载 `deploy` 发送 reference 与控制消息；
- 记录、回放和重采样 PICO motion；
- 用 Viser 实时显示实际发送的 G1 reference motion。

本分支不包含 checkpoint、ONNX、策略推理、MuJoCo sim2sim、G1 电机控制或
`g1_sim2real` bridge。策略部署内容只存在于 `deploy` 分支。

## 数据链路

```text
PICO → XRoboToolkit PC Service → Python SDK → G1 retarget
                                              ├─ 28701 请求
                                              ├─ 28702 36维 qpos
                                              ├─ 28703 手柄控制
                                              └─ 8080 Viser
```

`28701/28702/28703` 是可配置的 ZMQ TCP 端口。外部主机监听这些端口，G1 机载端主动连接，
无线网络上不传输 observation、policy action 或电机命令。

## 安装

```bash
cd sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

XRoboToolkit PC Service 需要单独安装，默认入口是
`/opt/apps/roboticsservice/runService.sh`。Python binding 的安装与验证见
[`sim2real/teleop/README.md`](sim2real/teleop/README.md)。

## 实时 PICO 启动

```bash
cd sim2real
SERVER_IP=192.168.31.20 \
REFERENCE_BIND_IP=0.0.0.0 \
VR_REQ_PORT=28701 VR_POSE_PORT=28702 VR_CTRL_PORT=28703 \
VIEWER_PORT=8080 \
XR_SERVICE_DIR=/opt/apps/roboticsservice \
bash scripts/launch_pico.sh --source pico
```

脚本只创建本机的 `xr-service` 和 `reference` 两个 tmux window，不使用 SSH，也不会启动
G1 上的任何程序。浏览器访问 `http://192.168.31.20:8080` 查看 reference。

## Motion 播放

```bash
cd sim2real
SERVER_IP=192.168.31.20 \
REFERENCE_BIND_IP=0.0.0.0 \
VR_REQ_PORT=28701 VR_POSE_PORT=28702 VR_CTRL_PORT=28703 \
MOTION_SELECT_PORT=28704 VIEWER_PORT=8080 \
bash scripts/launch_pico.sh --source motion
```

脚本默认自动扫描仓库的 `sim2real/config/g1/motions`，不需要指定 motion 路径，并打开
`motion-select` 交互窗口。机载 deploy 启动后只需按一次遥控器 `A` 进入策略控制；此后在
`motion-select` 输入编号或名称，motion 会在策略就绪后自动开始。动作结束时仍由当前策略
平滑过渡到默认 reference，并保持策略控制等待下一次选择，不需要再按 `Up` 或 `A`。
`28704` 仅供外部主机本地选择器使用，默认绑定 `127.0.0.1`。如需使用另一目录，可设置
`MOTION_ROOT=/path/to/motions`。

分支保留了原仓库全部 31 个 motion 文件。当前 reference server 面向 G1，因此只有包含完整
G1 joint names 的 robot motion 可以直接发送；L7 motion 作为原始 motion 数据保留。

停止服务：

```bash
bash scripts/launch_pico.sh --stop
```

更完整的启动参数见 [`PICO_BRANCH_zh.md`](PICO_BRANCH_zh.md)。
