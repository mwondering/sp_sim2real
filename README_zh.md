# G1 SPV5-2 Deploy 分支

本分支只保留 G1 SPV5-2 的 sim2sim 与 sim2real 部署。策略固定为：

```text
ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx
```

所有 motion 和 PICO/retarget 服务位于配套的 `pico` 分支。外部服务器通过三路 ZMQ TCP
发送 reference，机载端只负责接收 reference、读取机器人状态、拼接 observation 和执行 ONNX
推理。策略与 bridge 之间的 `55001/55002` UDP 始终走本机回环，电机命令不经过 Wi-Fi。

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

## G1 机载 sim2real

首次在 G1 ARM64 上构建 bridge：

```bash
cd g1_sim2real
G1_BRIDGE_BUILD_DIR=build_onboard bash scripts/build.sh
```

构建完成后，在 `deploy` 分支用一个脚本启动机载侧的两个 tmux window：

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

配套 `pico` 分支使用 `--source motion` 时，motion 单次播放，机载端使用 G1 遥控器完成
状态切换：

1. 默认位姿下按 `A`，开始外部主机当前选中的 motion；
2. motion 播放完成后按方向键 `Up`，机器人平滑回到默认位姿；
3. 外部主机在 `motion-select` 窗口选择下一条 motion，再按 `A` 开始。

只有收到外部服务器的 `finished` 状态且 reference 缓冲已经播放完，`Up` 才会生效。运动中
误按 `Up` 会被忽略；遥控器原有停止键仍退出策略并进入阻尼，不改变其安全语义。

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

配套的 `pico` 分支使用 `sim2real/scripts/launch_pico.sh` 启动外部主机侧的
`xr-service` 和 `reference`。两台机器各启动一个分支脚本，总计四个 tmux window。
