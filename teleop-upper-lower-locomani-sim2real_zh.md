# teleop-upper-lower-locomani 真机启动与操作

本文只描述已经通过 sim2sim 的新任务。原 `deploy.py`、原 PICO 端口和原 `g1_bridge.yaml` 均未改变。
通用机载拓扑、ARM64 安装和任务状态矩阵见 [G1 机载部署总说明](onboard-deployment_zh.md)。

## 1. 已确认的部署合同

- D435i 安装在 pelvis 附近，向下倾斜约 60°。
- 配置位姿为 `[0.12546, 0.0197, -0.05952] m`、下倾 `60°`，与 MJLab/XML 相机挂载一致。
- D435i 独立进程以 `640x360@30 Hz` 采集并发布原始 Z16 深度；策略进程在接收端生成
  `(1,36,64)` 输入。
- 策略/真机状态频率为 50 Hz；相机超时阈值为 0.25 秒。
- 初始模式为全身遥操作；PICO 右手 B 进入上下肢分离模式，左手 Y 返回全身模式，两个按键均为上升沿触发。
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

G1 的 Ubuntu 20.04 系统使用 glibc 2.31，而 PyPI 当前提供的 Python 3.12/AArch64
`pyrealsense2` wheel 会要求 `GLIBC_2.38`。不要单独替换 `/lib/aarch64-linux-gnu`
下的 glibc；应在 G1 上从源码构建只安装到上述虚拟环境的绑定：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  libssl-dev libusb-1.0-0-dev libudev-dev

test -d /home/unitree/librealsense-2.57.7 || \
  git clone --depth 1 --branch v2.57.7 \
    https://github.com/realsenseai/librealsense.git \
    /home/unitree/librealsense-2.57.7

cmake \
  -S /home/unitree/librealsense-2.57.7 \
  -B /home/unitree/librealsense-2.57.7/build-g1-py312 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=/home/unitree/g1-camera-stream/.venv/bin/python \
  -DBUILD_SHARED_LIBS=OFF \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_DDS=OFF

cmake --build /home/unitree/librealsense-2.57.7/build-g1-py312 \
  --target pyrealsense2 --parallel "$(nproc)"

uv pip uninstall --python .venv/bin/python pyrealsense2
cp -L \
  /home/unitree/librealsense-2.57.7/build-g1-py312/Release/pyrealsense2.cpython-312-aarch64-linux-gnu.so \
  .venv/lib/python3.12/site-packages/
```

若以后再次执行 `uv pip install -r requirements.txt`，PyPI wheel 会覆盖源码版本，需要重新执行
上面最后两条命令。验证 ABI、Python 导入和相机枚举：

```bash
getconf GNU_LIBC_VERSION
.venv/bin/python -c \
  'import pyrealsense2 as rs; print(rs.__file__, rs.pipeline); print([(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()])'
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
export ROBOT_IP="192.168.123.164"
export ROBOT_SSH="unitree@${ROBOT_IP}"
export G1_CAMERA_ROOT=/home/unitree/g1-camera-stream

ssh -t "${ROBOT_SSH}" \
  "cd '${G1_CAMERA_ROOT}' && \
   exec .venv/bin/python depth_camera_sender.py"
```

可在策略服务器的另一个终端处理并显示原始深度，不影响相机发布和策略：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
export ROBOT_IP="192.168.123.164"
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
G1_NET=enp4s0 \
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

必须看到 `command_timeout_monitor=0.12s timeout_action=warn-only`。命令中断超过
0.12 秒时只打印红色警告，不发送 damping。bridge 不再校验 `q_des/qd_des/kp/kd`
的有限性、范围或相邻命令跳变，也不再有 safety latch；格式异常的命令只记录并丢弃。

### 终端 5：双策略任务

HEFT：

```bash
cd /home/lenovo/workspace/UNICTL/motion_tracking_sim2real_self/sim2real
export ROBOT_IP="192.168.123.164"
uv run src/teleop_upper_lower_locomani.py \
  --target real \
  --whole-body-policy heft \
  --terrain-class 1 \
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

返回全身模式前仍须让所有摇杆回中并保持 0.3 秒，再按 PICO 左手 `Y`。右手 `B` 和左手 `Y` 都只在按下瞬间触发；长按不会重复切换，同一控制周期同时按下会忽略。模式切换会在 1.0 秒内混合关节目标和增益。

## 6. 按键

| 设备 | 输入 | 功能 |
| --- | --- | --- |
| G1 遥控器 | `start` | 从零力矩进入默认姿态 |
| G1 遥控器 | `A` | 从默认姿态进入策略 |
| G1 遥控器 | `select/stop` | 结束高层任务 |
| PICO 右手 | `A` | 启动或重新对齐 PICO |
| PICO 右手 | `B` | 进入上下肢分离（dual）模式，上升沿触发 |
| PICO 左手 | `Y` | 返回全身（whole-body）模式，上升沿触发 |
| PICO 左手 | `X` | 软件停止：立即发送 damping 并结束高层任务 |
| PICO 左摇杆 Y | 前后速度 | 楼梯模式只接受向前 |

楼梯模式会强制 `vy=0、wz=0`；左右摇杆的横移/转向输入不会生效。

PICO X 依赖 PICO、Wi-Fi、XRoboToolkit、ZMQ 和高层 Python 进程正常工作，
不是独立或安全认证的硬件急停。机器人失稳、软件无响应或通信中断时，仍须优先使用
G1 实体遥控器停止键。

## 7. 失效行为

- 全身模式 PICO 超时、双策略模式相机/PICO 超时、IMU 超限：终端打印红色警告，
  不进入 `safe-hold`。双策略输入断流时临时发送全身策略指令，输入恢复后继续双策略。
- 收到 0.25 秒内的新鲜 PICO 左手 X 输入：立即发送 damping 并结束高层任务。
- UDP 命令超过 0.12 秒中断：C++ bridge 只打印红色警告，不发送 damping。
- 高层进程正常退出或捕获到异常：关闭高层资源，不再自动发送 damping。
- NaN/Inf、数值越界和目标跳变不会触发 bridge 拒绝或 damping；格式异常的命令只记录并丢弃。
- 任意异常或机器人失稳：优先按 G1 遥控器停止键；PICO 左手 X 只作为辅助软件停止。

## 8. 正常停止

1. 停止移动，全部摇杆回中。
2. 如需高层显式发送 damping，按 PICO 左手 X；G1 遥控器 `select/stop` 只结束高层任务，不再自动发送 damping。
3. `Ctrl+C` 停止 bridge 时，bridge 自身的关闭路径仍会发送一次 damping。
4. 停止相机和 PICO 服务。

bridge 不再产生 `SAFETY LATCH`。命令超时只打印红色
`WARNING: valid UDP command timeout`，格式异常的命令打印后丢弃。

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

按 PICO B/Y 无法切换：

- 先按 PICO A；
- 确认相机进程持续发布；
- 四个摇杆全部回中至少 0.3 秒；
- 确认机器人没有明显倾斜或快速转动；
- 进入 dual 使用右手 B，返回 whole-body 使用左手 Y；松开对应按键后重新按一次。

任务提示 `No bridge state for 1s`：

- 真机任务会主动退出，但不再自动发送 damping；
- 检查 `G1_NET`、DDS LowState 和 UDP 端口；
- 排除原因后按完整顺序重启 bridge 与任务。

## 10. G1 全机载部署（实验性）

第 4 节是当前明确记录的真机流程：PICO 重定向、策略和 bridge 在策略服务器运行，G1 机载侧只
运行 D435i 发送端。把所有进程迁移到 G1 在代码和 localhost 地址上是可行的，但尚未完成
“XR 服务 + 重定向 + D435i + 三个 ONNX session + DDS bridge”并发负载和真机闭环验证，因此
本节只能作为实验性机载流程，不能宣称与第 4 节具有相同验证等级。

### 10.1 首次准备

1. 在 G1 上保留第 2 节的独立 `g1-camera-stream` Python 3.12 环境。
2. 把完整主仓库复制到 G1，在 `<repo>/sim2real` 执行 `uv sync`，随后执行
   `bash install_xrobottoolkit_sdk.sh`。
3. 安装并验证 XRoboToolkit ARM64/headless PC Service；PICO 客户端连接 G1 无线 IP。
4. 确认 `lower.onnx`、`upper.onnx` 以及所选 whole-body checkpoint 全部存在。
5. 在 G1 上重新编译专用 bridge：

   ```bash
   cd <repo>/g1_sim2real
   G1_BRIDGE_BUILD_DIR=build_locomani_onboard bash scripts/build.sh
   ```

不要复用外部 x86-64 策略机产生的 build 目录。全机载时
`teleop-upper-lower-locomani-real.yaml` 的深度地址保持 `tcp://127.0.0.1:28811`，PICO ZMQ
保持 `127.0.0.1:28701-28703`，策略/bridge UDP 保持 `127.0.0.1:55001-55002`。

### 10.2 机载启动顺序

终端 1：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

终端 2，启动本机 D435i；多相机时追加 `--serial-number`：

```bash
cd /home/unitree/g1-camera-stream
taskset -c 0 .venv/bin/python depth_camera_sender.py
```

终端 3，使用只绑定回环地址的 PICO 服务：

```bash
cd <repo>/sim2real
taskset -c 1 bash scripts/run_pico_server.sh
```

终端 4：

```bash
cd <repo>/g1_sim2real
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_locomani_onboard \
G1_BRIDGE_CONFIG=config/g1_bridge_teleop_upper_lower_locomani.yaml \
taskset -c 2-3 bash scripts/run_bridge.sh
```

终端 5，首次使用 HEFT，并直接采用任务 YAML 的 localhost 深度地址：

```bash
cd <repo>/sim2real
taskset -c 4-7 uv run src/teleop_upper_lower_locomani.py \
  --target real \
  --whole-body-policy heft \
  --terrain-class 1
```

只有在 SPV5-2 已完成相同 sim2sim 和吊装验证后，才把 `--whole-body-policy` 改为 `spv5_2`。
全机载时不再设置 `ROBOT_IP`，也不再传 `--depth-connect tcp://<外部IP>:28811`。

### 10.3 验收与回退

按第 5 节进入控制前，必须确认 PICO frame age、D435i frame age、bridge state interval 和策略
周期均稳定。若持续出现 `pico_stale`、`camera/depth_stale`、`No bridge state for 1s`、明显周期
抖动或机载过热，应立即停止并退回第 4 节外部策略服务器方案；不要放宽 0.25 秒超时掩盖负载
不足。专用 bridge 的命令超时动作仍是 warn-only，机载部署不会改变这一安全边界，实体遥控器和
硬件急停必须始终可用。
