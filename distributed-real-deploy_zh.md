# PICO/策略服务器 + 机器人相机/bridge 分布式部署

本方案用于以下固定角色划分：

- 服务器：XRoboToolkit PC Service、PICO 重定向、上下肢策略推理。
- 机器人计算机：D435i USB3 采集与预处理、G1 DDS bridge。
- G1 DDS 只存在于机器人侧；服务器与机器人计算机之间只传输 50 Hz 状态/指令和
  30 Hz 的 `(1,36,64)` 深度张量。

下文以服务器 `10.42.0.1`、机器人计算机 `10.42.0.2` 为例。实际地址通过环境变量传入，
不需要修改 YAML。

## 网络和端口

机器人计算机应至少有两条独立链路：

1. `G1_NET`：连接 G1 的 `192.168.123.x` DDS 网卡。
2. `ROBOT_CONTROL_IP`：连接策略服务器的专用有线网卡。

跨机端口只有：

| 端口 | 方向 | 内容 |
| --- | --- | --- |
| UDP 55001 | 机器人 → 服务器 | LowState、IMU、遥控器输入 |
| UDP 55002 | 服务器 → 机器人 | 关节目标和增益 |
| TCP 28811 | 机器人 → 服务器 | 处理后的策略深度 |

PICO 重定向端口 `28701/28702/28703` 绑定服务器回环地址，不向机器人开放。

## 首次准备

两台机器使用同一版本的仓库。以下环境配置和编译命令只需在机器人计算机执行一次。

### 推荐：机器人一键配置

默认使用服务器 `10.42.0.1`、机器人 `10.42.0.2`，并自动选择具有
`192.168.123.x` 地址的 G1 DDS 网卡。在机器人仓库根目录只执行：

```bash
cd /home/unitree/motion_tracking_sim2real_self && bash setup_robot_distributed.sh
```

脚本会自动完成：

1. 补齐缺少的系统工具；
2. 创建 `sim2real/.venv_depth` 并安装最小相机依赖；
3. 自动选择 D435i direct 模式，或查找已有的 worker Python；
4. 检查当前 CPU 架构对应的 Unitree SDK2；
5. 编译 `g1_sim2real/build_locomani/g1_udp_bridge`；
6. 执行 Python、Shell 和 bridge 冒烟检查；
7. 生成 `robot_distributed.env`，后续相机和 bridge 启动脚本会自动读取。

如果 IP 或 DDS 网卡不是默认值，仍然只需执行一条命令：

```bash
bash setup_robot_distributed.sh \
  --server-ip 10.42.0.1 \
  --robot-ip 10.42.0.2 \
  --g1-net enp3s0
```

如果 G1 上只有某个已有 Python 能导入 `pyrealsense2`，使用：

```bash
bash setup_robot_distributed.sh \
  --g1-net enp3s0 \
  --worker-python /absolute/path/to/realsense/python
```

脚本可以安全重复执行。除非机器人系统工具已经由管理员预装，否则不要增加
`--skip-system-packages`。

### 手动配置与故障排查

以下分步命令只用于一键脚本失败后的定位，不是正常部署的必做步骤。

### 1. 安装机器人侧基础工具

机器人侧需要 Python 3.10、Python venv、C++17 编译器和 CMake。以下命令适用于能够从
系统软件源安装 Python 3.10 的 Ubuntu：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  python3.10 \
  python3.10-venv
```

如果机器人已经通过 Conda、uv 或系统环境提供 Python 3.10，可以跳过 Python 软件包安装，
并在下文把 `python3.10` 替换成实际解释器。

### 2. 配置机器人运行变量

假定仓库位于 `/home/unitree/motion_tracking_sim2real_self`，机器人终端执行：

```bash
export ROBOT_REPO_ROOT=/home/unitree/motion_tracking_sim2real_self
export SERVER_CONTROL_IP=10.42.0.1
export ROBOT_CONTROL_IP=10.42.0.2
export G1_NET=enp3s0
export G1_BRIDGE_BUILD_DIR=build_locomani
export ROBOT_DEPTH_PYTHON="${ROBOT_REPO_ROOT}/sim2real/.venv_depth/bin/python"
```

其中 `ROBOT_REPO_ROOT` 和 `G1_NET` 必须改成机器人实际路径和 G1 DDS 网卡。

### 3. 创建最小深度相机 Python 环境

机器人只负责相机采集、预处理和 ZMQ 发布，不需要安装 ONNX、MuJoCo、Pinocchio 或策略
checkpoint。执行：

```bash
cd "${ROBOT_REPO_ROOT}/sim2real"

python3.10 -m venv .venv_depth
.venv_depth/bin/python -m pip install --upgrade pip
.venv_depth/bin/python -m pip install \
  numpy \
  pyyaml \
  pyzmq
```

#### direct 模式

如果机器人架构和 Python 环境可以直接安装 `pyrealsense2`：

```bash
cd "${ROBOT_REPO_ROOT}/sim2real"
.venv_depth/bin/python -m pip install pyrealsense2
.venv_depth/bin/python -c \
  'import numpy, yaml, zmq, pyrealsense2; print("D435i direct environment: OK")'
```

然后检查相机：

```bash
command -v rs-enumerate-devices >/dev/null && rs-enumerate-devices
lsusb -t
```

`rs-enumerate-devices` 由 librealsense tools 提供；如果系统没有该命令，上面的 Python 导入
检查仍然有效。D435i 应连接 USB 3/SuperSpeed，而不是 `480M` 的 USB 2 链路。

#### worker 模式

如果机器人是 Jetson/aarch64，或者 `pyrealsense2` 只能在已有的 RealSense Python 环境中
导入，主 `.venv_depth` 不安装 `pyrealsense2`，只设置 worker：

```bash
export D435I_WORKER_PYTHON=/absolute/path/to/realsense/python

"${D435I_WORKER_PYTHON}" -c \
  'import numpy, pyrealsense2; print("D435i worker environment: OK")'
"${ROBOT_DEPTH_PYTHON}" -c \
  'import numpy, yaml, zmq; print("D435i main environment: OK")'
```

正式运行时把 `D435I_WORKER_PYTHON` 一并传给
`scripts/run_robot_depth_distributed.sh`。worker 与主进程必须在同一台机器人计算机上。

### 4. 编译机器人 bridge

仓库已经包含 x86_64 和 aarch64 的 Unitree SDK2 静态库。先确认当前架构存在对应文件：

```bash
cd "${ROBOT_REPO_ROOT}/g1_sim2real"
uname -m
test -f "third_party/unitree_sdk2/lib/$(uname -m)/libunitree_sdk2.a"
```

然后使用独立构建目录编译：

```bash
cd "${ROBOT_REPO_ROOT}/g1_sim2real"
G1_BRIDGE_BUILD_DIR=build_locomani bash scripts/build.sh
```

验证产物：

```bash
test -x build_locomani/g1_udp_bridge
build_locomani/g1_udp_bridge --help
```

预期帮助信息包含：

```text
--state-host
--cmd-bind-host
--cmd-allowed-host
```

### 5. 机器人侧环境冒烟检查

```bash
cd "${ROBOT_REPO_ROOT}"

"${ROBOT_DEPTH_PYTHON}" -m py_compile \
  sim2real/src/depth_camera_real.py \
  sim2real/src/runtime/d435i_source.py \
  sim2real/src/runtime/d435i_worker.py \
  sim2real/src/runtime/depth_pipeline.py \
  sim2real/src/runtime/zmq_stream.py

bash -n \
  sim2real/scripts/run_robot_depth_distributed.sh \
  g1_sim2real/scripts/run_robot_bridge_distributed.sh
```

## 服务器统一发起全部命令

相机和 bridge 仍实际运行在机器人计算机上，但可以在服务器终端通过 SSH 启动。以下
命令全部在服务器执行，共使用 6 个终端。各终端的环境变量不会自动共享，最省事的做法
是在每个服务器终端都先执行一次：

```bash
export SERVER_REPO_ROOT=/home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self
export ROBOT_REPO_ROOT=/home/unitree/motion_tracking_sim2real_self

export SERVER_CONTROL_IP=10.42.0.1
export ROBOT_CONTROL_IP=10.42.0.2
export ROBOT_SSH_USER=unitree
export ROBOT_SSH_HOST=10.42.0.2
export ROBOT_SSH="${ROBOT_SSH_USER}@${ROBOT_SSH_HOST}"
export ROBOT_G1_NET=enp3s0
```

`ROBOT_REPO_ROOT`、`ROBOT_SSH_USER` 和 `ROBOT_G1_NET` 必须按机器人实际环境修改。建议提前
配置 SSH 密钥，并在上机前验证：

```bash
ssh "${ROBOT_SSH}" \
  "test -d '${ROBOT_REPO_ROOT}' && hostname && ip -br -4 address"
ping -c 5 "${ROBOT_CONTROL_IP}"
```

### 服务器终端 1：XRoboToolkit PC Service

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

### 服务器终端 2：PICO 重定向

```bash
cd "${SERVER_REPO_ROOT}/sim2real"
bash scripts/run_pico_server.sh
```

### 服务器终端 3：通过 SSH 启动机器人 D435i

```bash
ssh -t "${ROBOT_SSH}" \
  "cd '${ROBOT_REPO_ROOT}/sim2real' && \
   export ROBOT_CONTROL_IP='${ROBOT_CONTROL_IP}' && \
   bash scripts/run_robot_depth_distributed.sh"
```

如果 D435i 使用 worker，在服务器终端先增加：

```bash
export D435I_WORKER_PYTHON=/absolute/path/on/robot/to/python
export D435I_SERIAL=<相机序列号>

ssh -t "${ROBOT_SSH}" \
  "cd '${ROBOT_REPO_ROOT}/sim2real' && \
   export ROBOT_CONTROL_IP='${ROBOT_CONTROL_IP}' && \
   export D435I_WORKER_PYTHON='${D435I_WORKER_PYTHON}' && \
   export D435I_SERIAL='${D435I_SERIAL}' && \
   bash scripts/run_robot_depth_distributed.sh"
```

### 服务器终端 4：验证机器人深度

```bash
cd "${SERVER_REPO_ROOT}/sim2real"
uv run python src/view_depth_stream.py \
  --connect "tcp://${ROBOT_CONTROL_IP}:28811"
```

必须确认画面、无效像素比例和安装朝向正常后，才能继续。

### 服务器终端 5：通过 SSH 启动机器人 bridge

```bash
ssh -t "${ROBOT_SSH}" \
  "cd '${ROBOT_REPO_ROOT}/g1_sim2real' && \
   export SERVER_CONTROL_IP='${SERVER_CONTROL_IP}' && \
   export ROBOT_CONTROL_IP='${ROBOT_CONTROL_IP}' && \
   export G1_NET='${ROBOT_G1_NET}' && \
   export G1_BRIDGE_BUILD_DIR=build_locomani && \
   bash scripts/run_robot_bridge_distributed.sh"
```

必须看到：

```text
Startup damping active while waiting for the first valid command
task safety: watchdog=0.12s startup_damping=on damping_hz=50
```

### 服务器终端 6：启动策略

```bash
cd "${SERVER_REPO_ROOT}/sim2real"
export SERVER_CONTROL_IP=10.42.0.1
export ROBOT_CONTROL_IP=10.42.0.2
bash scripts/run_server_policy_distributed.sh
```

第一条合法命令到达机器人后，bridge 必须打印：

```text
First valid command accepted; startup damping released and watchdog armed
```

然后才按原真机顺序使用 G1 实体遥控器 `start` 和 `A`。

### 从服务器检查进程和端口

```bash
# 服务器本地
pgrep -af 'serve_xrobot_teleop|teleop_upper_lower_locomani'
ss -ltnup | rg '28701|28702|28703|55001'

# 机器人侧，通过服务器 SSH 查询
ssh "${ROBOT_SSH}" \
  "pgrep -af 'depth_camera_real|g1_udp_bridge'; \
   ss -ltnup | grep -E '28811|55002'"
```

### 停止顺序

1. 全部摇杆回中，先用 G1 实体遥控器停止键或 PICO 左手 X 请求停止。
2. 在服务器策略终端按 `Ctrl+C`，确认 bridge 进入 damping。
3. 在 bridge 的 SSH 终端按 `Ctrl+C`。
4. 在深度进程的 SSH 终端按 `Ctrl+C`。
5. 最后停止 PICO 重定向和 XRoboToolkit PC Service。

不要先关闭 bridge 或机器人网络再停止策略。策略异常退出后，等待 bridge 的 0.12 秒
watchdog 锁定并确认机器人处于 damping，再结束其余进程。

## 按机器分别执行的启动顺序

### 1. 服务器：XRoboToolkit PC Service

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

PICO 应用填写服务器连接 PICO 的 Wi-Fi/LAN 地址；该地址不一定是 `SERVER_CONTROL_IP`。

### 2. 服务器：PICO 重定向

```bash
cd sim2real
bash scripts/run_pico_server.sh
```

### 3. 机器人：D435i 深度

```bash
cd sim2real
export ROBOT_CONTROL_IP=10.42.0.2
bash scripts/run_robot_depth_distributed.sh
```

如果使用 worker：

```bash
export D435I_WORKER_PYTHON=/absolute/path/to/python
export D435I_SERIAL=<相机序列号>
bash scripts/run_robot_depth_distributed.sh
```

### 4. 服务器：验证深度

```bash
cd sim2real
uv run python src/view_depth_stream.py \
  --connect tcp://10.42.0.2:28811
```

必须确认画面、无效像素比例和安装朝向正常后，才能继续。

### 5. 机器人：G1 bridge

```bash
cd g1_sim2real
export SERVER_CONTROL_IP=10.42.0.1
export ROBOT_CONTROL_IP=10.42.0.2
export G1_NET=enp3s0
export G1_BRIDGE_BUILD_DIR=build_locomani
bash scripts/run_robot_bridge_distributed.sh
```

bridge 收到 LowState 后会持续发送启动阻尼。必须看到：

```text
Startup damping active while waiting for the first valid command
task safety: watchdog=0.12s startup_damping=on damping_hz=50
```

### 6. 服务器：策略

```bash
cd sim2real
export SERVER_CONTROL_IP=10.42.0.1
export ROBOT_CONTROL_IP=10.42.0.2
bash scripts/run_server_policy_distributed.sh
```

第一条合法命令到达机器人后，bridge 打印：

```text
First valid command accepted; startup damping released and watchdog armed
```

然后才按原真机顺序使用 G1 实体遥控器 `start` 和 `A`。

## 失效行为

- 第一条合法策略命令前：bridge 持续发送 damping。
- 合法命令中断超过 0.12 秒：bridge 锁定并持续发送 damping。
- 非 `SERVER_CONTROL_IP` 来源的 UDP 指令：bridge 在解码前拒绝，并累计
  `source_rejected`。
- 深度超过 0.25 秒未到达服务器：双策略进入 safe-hold。
- PICO 左手 X 是软件停止，仍不替代 G1 实体遥控器停止键。

策略进程重启会使 UDP 序列号从零开始。为避免旧 bridge 拒绝重启后的低序号命令，
策略异常退出或 safety latch 后应同时重启 bridge，并重新执行完整上机顺序。

## 防火墙建议

只允许两个固定地址互访：

- 服务器只接受来自机器人 IP 的 UDP 55001。
- 机器人只接受来自服务器 IP 的 UDP 55002。
- 机器人只允许服务器 IP 连接 TCP 28811。

首次吊架测试必须实际拔掉服务器网线，并确认 bridge 在 0.12 秒内锁定且持续输出
damping。不要用增大 watchdog 超时的方法掩盖网络丢包或抖动。
