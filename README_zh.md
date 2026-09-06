# G1 SPV5-2 Deploy 分支

本分支提供 G1 SPV5-2 的 sim2sim 与 sim2real 部署，并参考 MimicLite 支持把
XRoboToolkit PC Service、PICO 接收、G1 retarget、策略推理和 bridge 全部放在 G1
机载计算机上。默认策略为：

```text
ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx
```

同时保留原始的兼容策略及其 JSON 元数据，便于切换和对照测试：

```text
ckpts/0729_baoshou_waist_dataclean_changedr_nohandxml/policy_28000.onnx
```

为了兼容已有流程，默认仍是外置 PICO reference；显式传入 `--onboard-pico` 才启用机载
PICO。机载时 reference 三路 ZMQ 和策略/bridge 两路 UDP 都走 `127.0.0.1`，Wi-Fi 只承载
PICO 到 G1 上 XR Service 的原始 XR 数据以及浏览器查看 MJViser，不再承载 retarget 后的
逐帧策略 reference。

## MimicLite 方式的机载 PICO、retarget 与可视化

部署结构如下：

```text
PICO --Wi-Fi/LAN--> XR Service --local SDK--> retarget + MJViser
                                             | 127.0.0.1:28701-28703
                                             v
                                      SPV5-2 policy
                                             | 127.0.0.1:55001/55002
                                             v
                                      Unitree C++ bridge
```

与 MimicLite 一样，策略环境和 PICO/retarget 环境分离。先准备策略环境，再构建独立的
`venv/pico`：

```bash
cd sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

安装脚本会执行 `uv sync --project venv/pico`，并在当前架构上从源码构建
`xrobotoolkit_sdk`。Unitree G1 使用 Ubuntu 20.04 / JetPack 5，XRoboToolkit PC Service
必须使用 [MimicLite shared artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
中的兼容包，不能使用上游 generic/headless ARM64 包：

```text
third_party/prebuilt/jetpack5-aarch64/xrobotservice/
  XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb
```

把兼容包放到上述目录，然后安装；也可以把 `.deb` 的实际路径作为第一个参数传入：

```bash
cd sim2real
bash install_xrobottoolkit_pc_service.sh --download
# 或：bash install_xrobottoolkit_pc_service.sh /path/to/package.deb
```

`--download` 使用固定 Google Drive 文件 ID，并校验约 68 MB 归档和内部 `.deb` 的
SHA-256；无参数时优先使用上述本地文件，文件不存在则自动进入相同的下载流程。G1 需要
走反向代理时可加：

```bash
HTTPS_PROXY=http://127.0.0.1:17890 \
bash install_xrobottoolkit_pc_service.sh --download
```

安装器会在覆盖 `/opt/apps/roboticsservice` 之前解包，并使用 G1 本机动态加载器检查 ICU、
GLIBC 和 GLIBCXX。若仍依赖 `libicuuc.so.70` 或 `GLIBC_2.34`，安装会被拒绝。不要将
`libicuuc.so.66` 伪装链接为 `.so.70`。安装后确认下面的入口存在：

```bash
test -f /opt/apps/roboticsservice/runService.sh
test -x /opt/apps/roboticsservice/RoboticsServiceProcess
venv/pico/.venv/bin/python -c \
  'import xrobotoolkit_sdk, mujoco, mink, mjviser; print("PICO runtime: OK")'
```

首次仍需构建 G1 bridge：

```bash
cd ../g1_sim2real
G1_BRIDGE_BUILD_DIR=build_onboard bash scripts/build.sh
```

然后在 G1 上一条命令启动四个 tmux window：

```bash
cd ../sim2real
VIEWER_URL_HOST=<G1局域网IP> \
G1_DDS_IFACE=eth0 \
bash scripts/launch_deploy.sh --real --onboard-pico
```

四个窗口分别是 `xr-service`、`reference`、`bridge`、`policy`。PICO 应连接 G1 上运行的
XR Service。浏览器打开 `http://<G1局域网IP>:8080`，可同时查看人体坐标轴和 retarget 后
的 G1；确认动作、脚底高度和朝向正确后，再按原有遥控器流程进入策略控制。
launcher 会直接以前台方式运行 `RoboticsServiceProcess`，避免官方 `runService.sh` 将
服务放入后台后立即返回、导致 tmux 显示 `Pane is dead (status 0)`；此时 Ctrl-C 也会
直接停止 XR Service。创建 tmux 前还会执行动态依赖预检；不兼容时会直接指出缺失库和
Ubuntu 20.04 兼容包名称，不再等到 pane 以 `status 127` 退出。
launcher 在创建 tmux window 时直接执行 `bash --noprofile --norc`，因此不会触发 Unitree
系统交互式 shell 中的 `ros:foxy(1) noetic(2) ?`，也不会再把部署命令误当作 ROS 选项。

当前操作员标定身高已设为 `1.80 m`。脚底高度采用 MimicLite 相同思路：启动前 30 帧标定
固定 Z 偏移，目标最小脚部高度为 `0.01 m`，避免每帧强行贴地造成跳动。

机载模式默认使用：

```text
RETARGET_LOOKBACK_MS=50.0
RETARGET_CPU_SET=0-1
BRIDGE_CPU_SET=2-3
POLICY_CPU_SET=4-7
```

机载 PICO 默认恢复 `50 ms` retarget lookback，对关节位置做线性插值、对根姿态做 Slerp。
后来增加的可配置 reference FIFO 延迟缓冲已经移除。策略自身仍要求保留 7 个未来帧
（约 `0.14 s`），这是 ONNX 输入的前视窗口，不是额外的延迟 buffer。测最低延迟时可显式
设置 `RETARGET_LOOKBACK_MS=0`；不需要查看动作时可加 `--no-viewer`。MJViser 默认仅以 5 Hz
刷新，不参与 50 Hz 策略控制环。

## 带 MJViser 的机载 motion 回放

新增的 `motion-vis` 模式让 reference server、selector、policy 都在 G1 上，并确保浏览器看到
的 qpos 就是发给策略的 qpos：

```bash
cd sim2real
MOTION_ROOT=../motion VIEWER_URL_HOST=<G1局域网IP> \
bash scripts/launch_deploy.sh --real --source motion-vis
```

脚本创建 `reference`、`bridge`、`policy`、`motion-select` 四个窗口，不启动 XR Service。
浏览器仍访问 `http://<G1局域网IP>:8080`。如果只追求最低播放开销、不需要可视化，继续用
原来的 `--source motion`。

## 网络变量

```bash
export G1_REF_HOST=192.168.31.20
export G1_REF_REQ_PORT=28701
export G1_REF_POSE_PORT=28702
export G1_REF_CTRL_PORT=28703

export G1_STATE_PORT=55001
export G1_CMD_PORT=55002
```

也可以使用完整地址覆盖单个通道：`G1_REF_REQ_ADDR`、`G1_REF_POSE_ADDR`、
`G1_REF_CTRL_ADDR`。

## 非机载主机 sim2sim 可视化

先在 `pico` 分支启动 reference server，然后在本分支运行：

```bash
# 终端 1：MuJoCo 可视化
cd sim2real
uv run src/sim2sim.py --robot g1

# 终端 2：SPV5-2 policy
cd sim2real
G1_REF_HOST=127.0.0.1 \
G1_REF_REQ_PORT=28701 \
G1_REF_POSE_PORT=28702 \
G1_REF_CTRL_PORT=28703 \
uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_2.yaml
```

MuJoCo 窗口保持原键盘流程：`s` 进入默认姿态，等待 grounded PD hold 后按 `a`，按 `x`
停止。reference motion 的网页可视化由 `pico` 分支提供。

## G1 机载 sim2real（外置 PICO 兼容模式）

首次在 G1 ARM64 上构建 bridge：

```bash
cd g1_sim2real
G1_BRIDGE_BUILD_DIR=build_onboard bash scripts/build.sh
```

构建完成后，可继续用原来的外置 PICO 方式启动机载侧两个 tmux window：

```bash
cd sim2real
REFERENCE_HOST=192.168.31.20 \
VR_REQ_PORT=28701 VR_POSE_PORT=28702 VR_CTRL_PORT=28703 \
STATE_PORT=55001 CMD_PORT=55002 \
G1_DDS_IFACE=eth0 \
bash scripts/launch_deploy.sh --real
```

这两个 window 分别运行 `bridge` 和 `policy`。以下分开启动命令仅用于排障。

## 外部 motion 单次播放

配套 `pico` 分支使用 `--source motion` 时，motion 单次播放，但机载策略持续运行：

1. 机载 deploy 启动后，在默认位姿按一次 `A` 进入 SPV5-2 策略；
2. 外部主机在 `motion-select` 选择 motion，机载端在默认 reference 就绪后自动执行；
3. motion 播放完成后，机载端在当前策略内自动追加 100 步默认 reference 过渡；
4. 默认过渡完成后继续保持策略控制并等待，收到下一条 motion 后自动执行。

`A` 只需按一次，不需要按住，也不需要用 `Up` 返回默认位姿。新的 motion 可以在默认过渡
期间排队，但只会在过渡完成后启动。遥控器原有停止键仍退出策略并进入阻尼，不改变其安全
语义。

启动 bridge：

```bash
cd g1_sim2real
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_onboard \
taskset -c 2-3 bash scripts/run_bridge.sh \
  --state-host 127.0.0.1 --state-port 55001 \
  --cmd-bind-host 127.0.0.1 --cmd-port 55002 \
  --cmd-allowed-host 127.0.0.1
```

启动策略：

```bash
cd sim2real
G1_REF_HOST=192.168.31.20 \
G1_REF_REQ_PORT=28701 \
G1_REF_POSE_PORT=28702 \
G1_REF_CTRL_PORT=28703 \
G1_STATE_PORT=55001 G1_CMD_PORT=55002 \
taskset -c 4-7 uv run src/deploy.py \
  --robot g1 --tracking-config tracking_spv5_2.yaml
```

配套的 `pico` 分支仍可使用 `sim2real/scripts/launch_pico.sh` 启动外部主机侧的
`xr-service` 和 `reference`。这条兼容路径仍需两台机器；新部署建议使用上面的
`--onboard-pico`。

## 机载 motion 模式（延迟对照）

无线 PICO 模式仍是默认值；只有显式添加 `--source motion` 才启用机载播放：

```bash
cd sim2real
STATE_PORT=55001 \
CMD_PORT=55002 \
G1_DDS_IFACE=eth0 \
G1_BRIDGE_BUILD_DIR=build_onboard \
bash scripts/launch_deploy.sh --real --source motion
```

该模式不需要设置 `REFERENCE_HOST`。脚本会创建 `bridge`、`policy` 和 `motion-select`
三个 tmux window，并自动进入 `motion-select` 窗口。motion 默认递归查找仓库根目录
`motion/`；如需覆盖，可设置 `MOTION_ROOT=/absolute/path/to/motion`。

机器人到达默认位姿后按遥控器 `A` 进入 tracking，然后在 selector 中输入编号或名称。
选择命令仅通过机载 `127.0.0.1:28562` 发送一次；NPZ 在 policy 进程中按需加载，后续逐帧
播放不经过 Wi-Fi。选择可以在当前 motion 或默认过渡期间排队；当前 motion 完成后会自动
回到默认 reference，并在默认过渡完成后启动最新选择。

恢复原无线模式时不传 `--source motion`，或者显式运行：

```bash
REFERENCE_HOST=<pico-host-ip> bash scripts/launch_deploy.sh --real --source pico
```
