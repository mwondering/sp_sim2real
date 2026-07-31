# teleop-upper-lower-locomani 真机启动与操作

本文只描述已经通过 sim2sim 的新任务。原 `deploy.py`、原 PICO 端口和原 `g1_bridge.yaml` 均未改变。

## 1. 已确认的部署合同

- D435i 安装在 pelvis 附近，向下倾斜约 60°。
- 配置位姿为 `[0.12546, 0.0197, -0.05952] m`、下倾 `60°`，与 MJLab/XML 相机挂载一致。
- D435i 独立进程以 `640x360@30 Hz` 采集并发布原始 Z16 深度；策略进程在接收端生成
  `(1,36,64)` 输入。
- 策略/真机状态频率为 50 Hz；相机超时阈值为 0.25 秒。
- 初始模式为全身遥操作；PICO 右手 B 在全身与上下肢分离模式间切换。
- PICO 左手 X 是软件停止键：控制包有效时立即发送 damping 并结束高层任务。
- 楼梯模式 `terrain-class=2` 只允许前进，不支持横移、倒退和原地旋转。高度摇杆不启用。

## 2. 首次准备

Python 环境：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

需要单独显示深度图时，再安装：

```bash
uv pip install opencv-python
```

G1 机载计算机只需克隆独立相机分支：

```bash
git clone --branch g1-camera-stream --single-branch \
  https://github.com/mwondering/sp_sim2real.git /home/unitree/g1-camera-stream
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

该分支只包含 D435i 读取和 ZMQ 发送端，不包含策略、bridge、ONNX、MuJoCo 或 PICO。
统一环境固定使用 Python 3.12。

按原 HEFT 流程编译 bridge：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/g1_sim2real
bash scripts/build.sh
```

如果该仓库是复制得到的，旧 `build/` 可能记录其他路径。不要删除旧目录，改用：

```bash
G1_BRIDGE_BUILD_DIR=build_locomani bash scripts/build.sh
```

## 3. 上机前检查

确认 ONNX：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
test -f config/g1/ckpts/MJLab_Locomani/lower.onnx
test -f config/g1/ckpts/MJLab_Locomani/upper.onnx
```

在策略服务器设置 G1 SSH 地址及相机分支路径，并确认 D435i：

```bash
export ROBOT_IP="<G1机载计算机的192.168.123.164>"
export ROBOT_SSH="unitree@${ROBOT_IP}"
export G1_CAMERA_ROOT=/home/unitree/g1-camera-stream

ssh "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   .venv/bin/python -c \
   'import pyrealsense2 as rs; print(rs.__file__, rs.pipeline)'"
```

确认 G1 有线网卡，例如：

```bash
ip -br address
```

下文用 `enp3s0` 表示 G1 DDS 网卡，实际运行时必须替换。

首次真机运行必须使用可靠吊架/保护绳，清空机器人周围人员和障碍物，并由一名操作者始终握住 G1 遥控器。

## 4. 完整启动顺序

共使用五个终端。

### 终端 1：XRoboToolkit 服务

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

打开 PICO XRoboToolkit 应用，完成控制器和身体追踪器连接、校准。

### 终端 2：PICO 重定向

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
uv run python teleop/serve_xrobot_teleop.py --robot g1
```

这仍是原 PICO 路线和原端口 `28701/28702/28703`。不要同时启动第二份 PICO 服务。

### 终端 3：D435i 独立深度进程

从策略服务器通过 SSH 启动 G1 上的独立相机分支。远端发布器默认监听
`tcp://*:28811`，只需知道 SSH 地址和分支绝对路径：

```bash
export ROBOT_IP="<G1机载计算机的192.168.123.x地址>"
export ROBOT_SSH="unitree@${ROBOT_IP}"
export G1_CAMERA_ROOT=/home/unitree/g1-camera-stream

ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py"
```

可在策略服务器的另一个终端处理并显示原始深度，不影响相机发布和策略：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
export ROBOT_IP="<G1机载计算机的192.168.123.x地址>"
uv run src/view_depth_stream.py \
  --config config/g1/teleop-upper-lower-locomani-real.yaml \
  --connect "tcp://${ROBOT_IP}:28811"
```

窗口中近处偏红、远处偏蓝黑，无效像素为黑色；按 `q` 或 `Esc` 只关闭查看器。相机进程
只读取并发送原始 Z16 帧，不再包含预处理或 GUI。

多相机时指定序列号：

```bash
ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py \
   --serial-number <D435I_SERIAL>"
```

正常日志应包含：

```text
[D435i] ... stream=640x360@30 ... fov=...deg
[G1Depth] D435i(...) -> tcp://*:28811 raw-z16 shape=(360,640) scale=...m/unit
```

相机进程会读取实际设备序列号、深度比例与内参，并校验
`640x360@30` 和训练视场角；校验不通过时会拒绝启动。
机器人侧发送 `uint16[360,640]` 原始帧和深度比例，策略所需的米制转换、无效值处理、
裁剪、缩放与归一化均在接收端完成。

### 终端 4：G1 底层 bridge

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/g1_sim2real
G1_NET=enp3s0 \
G1_BRIDGE_CONFIG=config/g1_bridge_teleop_upper_lower_locomani.yaml \
bash scripts/run_bridge.sh
```

若使用独立构建目录：

```bash
G1_NET=enp3s0 \
G1_BRIDGE_BUILD_DIR=build_locomani \
G1_BRIDGE_CONFIG=config/g1_bridge_teleop_upper_lower_locomani.yaml \
bash scripts/run_bridge.sh
```

必须看到 `task safety: watchdog=0.12s`。此 watchdog 只在收到第一条有效高层命令后生效；命令中断、NaN/Inf 或越界时会锁定 damping，恢复必须重启 bridge。

### 终端 5：双策略任务

HEFT：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
export ROBOT_IP="<G1机载计算机的192.168.123.x地址>"
uv run src/teleop_upper_lower_locomani.py \
  --target real \
  --whole-body-policy heft \
  --terrain-class 2 \
  --depth-connect "tcp://${ROBOT_IP}:28811"
```

SPV5-2：

```bash
uv run src/teleop_upper_lower_locomani.py \
  --target real \
  --whole-body-policy spv5_2 \
  --terrain-class 2 \
  --depth-connect "tcp://${ROBOT_IP}:28811"
```

首次只建议使用已经充分验证的 HEFT。不要在一次运行中切换 HEFT 与 SPV5-2。

## 5. 从阻尼到策略的操作顺序

这里不使用电脑键盘 `s/a`；真机使用 G1 实体遥控器，顺序与原实机 HEFT 部署一致。

1. 确认 D435i 画面正常、PICO 服务正常、bridge 持续收到 LowState。
2. 按 G1 遥控器 `start`，机器人用 2 秒进入默认姿态。
3. 等待机器人完全稳定。
4. 按 G1 遥控器 `A`，进入默认的全身策略。
5. 保持自然站立，按 PICO 右手 `A`，启动/重对齐 PICO。
6. 先在全身模式小幅验证上肢和身体跟随。
7. 所有 PICO 摇杆回中并保持至少 0.3 秒，机器人直立稳定后，按 PICO 右手 `B`，进入上下肢分离模式。
8. 上楼梯只使用左摇杆 Y 缓慢给前进命令；松开即为零速度命令。

再次切换前仍须让所有摇杆回中并保持 0.3 秒，再按 PICO 右手 B。切换会在 1.0 秒内混合关节目标和增益。

## 6. 按键

| 设备 | 输入 | 功能 |
| --- | --- | --- |
| G1 遥控器 | `start` | 从零力矩进入默认姿态 |
| G1 遥控器 | `A` | 从默认姿态进入策略 |
| G1 遥控器 | `select/stop` | 结束高层任务 |
| PICO 右手 | `A` | 启动或重新对齐 PICO |
| PICO 右手 | `B` | 切换两种策略模式 |
| PICO 左手 | `X` | 软件停止：立即发送 damping 并结束高层任务 |
| PICO 左摇杆 Y | 前后速度 | 楼梯模式只接受向前 |

楼梯模式会强制 `vy=0、wz=0`；左右摇杆的横移/转向输入不会生效。

PICO X 依赖 PICO、Wi-Fi、XRoboToolkit、ZMQ 和高层 Python 进程正常工作，
不是独立或安全认证的硬件急停。机器人失稳、软件无响应或通信中断时，仍须优先使用
G1 实体遥控器停止键。

## 7. 失效行为

- 双策略模式下相机或 PICO 超过 0.25 秒未更新：进入 `safe-hold`，不会自动切换策略。
- IMU 倾斜或角速度超过新任务配置阈值：进入 `safe-hold`。
- `safe-hold` 时若输入恢复：全部摇杆回中并保持 0.3 秒，按 PICO 右手 B，只恢复到全身模式。
- 收到 0.25 秒内的新鲜 PICO 左手 X 输入：立即发送 damping 并结束高层任务。
- 高层进程退出或 UDP 命令超过 0.12 秒中断：C++ bridge 锁定 damping；必须先确认原因，再重启 bridge。
- 任意异常或机器人失稳：优先按 G1 遥控器停止键；PICO 左手 X 只作为辅助软件停止。

## 8. 正常停止

1. 停止移动，全部摇杆回中。
2. 按 G1 遥控器 `select/stop`；也可以按 PICO 左手 X 请求软件停止。两者都会让任务退出并发送 damping。
3. 确认机器人进入阻尼/安全支撑状态。
4. `Ctrl+C` 停止任务、bridge、相机、PICO 服务。

bridge 的 watchdog 锁定后日志包含：

```text
[G1Bridge] SAFETY LATCH: ...; publishing damping and requiring bridge restart
```

这是预期保护，不能在当前 bridge 进程中解锁。

## 9. 快速排障

检查进程和端口：

```bash
pgrep -af 'serve_xrobot_teleop|g1_udp_bridge|teleop_upper_lower'
ss -ltnp | rg '28701|28702|28703'
ss -tnp | rg '28811'

export ROBOT_SSH="unitree@<G1机载计算机的192.168.123.x地址>"
ssh "${ROBOT_SSH}" \
  "pgrep -af depth_camera_sender; ss -ltnp | grep 28811"
```

相机窗口全黑：

- 确认镜头保护膜已移除、USB 3 连接正常；
- 在 G1 上用 `rs-enumerate-devices` 检查深度流；
- 查看终端 `invalid=`；持续超过 60% 时任务不会进入上下肢分离模式；
- 室外强光、反光/透明材料和过近物体会造成 D435i 无效深度。

按 PICO B 无法切换：

- 先按 PICO A；
- 确认相机进程持续发布；
- 四个摇杆全部回中至少 0.3 秒；
- 确认机器人没有明显倾斜或快速转动；
- 松开 B 后重新按一次。

任务提示 `No bridge state for 1s`：

- 真机任务会主动退出，随后 bridge watchdog 进入 damping；
- 检查 `G1_NET`、DDS LowState 和 UDP 端口；
- 排除原因后按完整顺序重启 bridge 与任务。
