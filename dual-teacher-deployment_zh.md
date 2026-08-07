# Dual-Teacher sim2sim / sim2real 部署说明

本文只描述 G1 Dual-Teacher 遥操作任务的启动方式。默认使用 `motion` 输入模式：PICO 姿态经过仓库已有的重定向、FK 和 tracking reference 链路进入 Dual-Teacher；当前启动流程不需要深度相机进程。

## 1. 配置与 checkpoint

sim2sim 和 sim2real 分别使用以下任务配置，启动命令中必须显式指定对应 YAML：

| 场景 | 任务配置 | 底层桥配置 |
| --- | --- | --- |
| sim2sim | `sim2real/config/g1/dual-teacher.yaml` | `sim2real/config/g1/bridge.yaml` |
| sim2real | `sim2real/config/g1/dual-teacher-real.yaml` | `g1_sim2real/config/g1_bridge.yaml` |

Dual-Teacher ONNX 已放入仓库：

```text
sim2real/config/g1/ckpts/DualTeacher/model_30000.onnx
```

两个任务 YAML 都使用相对于自身目录的路径：

```yaml
dual_onnx: "ckpts/DualTeacher/model_30000.onnx"
```

文件的 SHA-256 应为：

```text
c6f58ad3ea07f454257d463370b4105c815385a5532b934e111c711392cbeb77
```

可在仓库根目录校验：

```bash
sha256sum sim2real/config/g1/ckpts/DualTeacher/model_30000.onnx
```

首次使用先安装 Python 环境：

```bash
cd <repo>/sim2real
uv sync
```

下面的 `<repo>` 均指当前仓库根目录。

## 2. PICO 公共前置流程

sim2sim 和 sim2real 使用同一套 PICO 服务。先完成 XRoboToolkit 的头显、控制器和身体追踪器连接与标定。

### 终端 1：XRoboToolkit PC 服务

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

打开 PICO 上的 XRoboToolkit 应用，连接运行该服务的计算机，并确认身体追踪与控制器数据正常。

### 终端 2：PICO 重定向服务

```bash
cd <repo>/sim2real
uv run python teleop/serve_xrobot_teleop.py --robot g1
```

保持这两个进程运行，再启动下面的 sim2sim 或 sim2real 流程。

## 3. sim2sim 启动

### 终端 3：MuJoCo 底层桥

```bash
cd <repo>/sim2real
uv run src/sim2sim.py \
  --robot g1 \
  --bridge-config config/g1/bridge.yaml
```

### 终端 4：Dual-Teacher 策略

```bash
cd <repo>/sim2real
uv run src/dual_teacher_adapter.py \
  --robot g1 \
  --target sim \
  --task-config config/g1/dual-teacher.yaml \
  --controller-config config/g1/controller.yaml \
  --tracking-config tracking.yaml
```

### 进入控制

`s`、`a`、`x` 都要输入到运行 `sim2sim.py` 的终端，不是在 MuJoCo 图形窗口中输入。

1. 点击终端 3，使其获得键盘焦点，按 `s`，机器人移动到默认姿态。
2. 等待终端出现 `Default target is stable; grounded PD hold is active`。
3. 在终端 3 按 `a`，释放临时基座稳定器并进入策略控制。
4. 保持操作者自然站立，按 PICO 右手控制器 `A`，启动或重新对齐 PICO 姿态流。

进入策略后，tracking policy 先作为平衡 fallback。PICO 输入稳定后，策略终端应出现：

```text
[DualTeacher] inputs ready
[DualTeacher] switched -> dual
```

输入未就绪时出现下面的日志是正常的，机器人会继续使用 tracking fallback：

```text
[DualTeacher] tracking fallback: pico_stale
```

PICO 输入超过配置的超时时间后，控制器会从 Dual-Teacher 平滑切回 tracking；输入恢复并持续就绪后再平滑切回 Dual-Teacher。

停止 sim2sim 时，在终端 3 按 `x`；必要时再对其余进程按 `Ctrl+C`。

## 4. sim2real 启动

警告：首次实机运行该 checkpoint 时必须为机器人提供可靠保护，清空运动范围内的人员和障碍物，并确保操作者可以立即触发急停或阻尼状态。必须先完成 sim2sim 验证。

Python 策略和 C++ bridge 的默认 UDP 地址都是 `127.0.0.1`，因此下面默认二者运行在同一台计算机。`G1_NET` 是这台计算机连接 G1 的 DDS 网卡名，不是机器人 IP。

### 首次部署：编译底层桥

```bash
cd <repo>/g1_sim2real
bash scripts/build.sh
```

可用下面的命令确认网卡名：

```bash
ip -br address
```

### 终端 3：G1 底层桥

将 `enp3s0` 替换为实际连接 G1 的网卡：

```bash
cd <repo>/g1_sim2real
G1_NET=enp3s0 \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
bash scripts/run_bridge.sh
```

### 终端 4：Dual-Teacher 策略

```bash
cd <repo>/sim2real
uv run src/dual_teacher_adapter.py \
  --robot g1 \
  --target real \
  --task-config config/g1/dual-teacher-real.yaml \
  --controller-config config/g1/controller.yaml \
  --tracking-config tracking.yaml
```

### 进入控制

1. 确认机器人已可靠保护、底层桥已收到 LowState、策略进程已连接底层桥。
2. 按 G1 遥控器 `start`，使机器人进入默认姿态。
3. 等待默认姿态过渡完成，确认机器人安全落地并稳定站立。
4. 按 G1 遥控器 `A`，进入策略控制。此时仍先由 tracking fallback 保持平衡。
5. 保持操作者自然站立，按 PICO 右手控制器 `A`，启动或重新对齐 PICO 姿态流。
6. 确认策略终端依次出现 `[DualTeacher] inputs ready` 和 `[DualTeacher] switched -> dual`。

停止时使用 G1 遥控器的 `select` 或现场配置的停止键进入停止/阻尼状态；异常时直接使用硬件急停。随后依次结束策略、底层桥和 PICO 服务。

## 5. 按键速查

| 场景 | 按键 | 作用 |
| --- | --- | --- |
| sim2sim 终端 | `s` | 从零力矩等待进入默认姿态 |
| sim2sim 终端 | `a` | 释放临时稳定器并进入策略 |
| sim2sim 终端 | `x` | 停止 sim2sim |
| G1 遥控器 | `start` | 实机进入默认姿态 |
| G1 遥控器 | `A` | 实机进入策略 |
| PICO 右手控制器 | `A` | 启动或重新对齐 PICO 姿态流 |

## 6. 常见问题

### 一进入策略就向后倒

先确认使用的是本页的 Dual-Teacher adapter 和对应 YAML，而不是通用 `deploy.py` 或 upper/lower task。正确流程会先运行 tracking fallback，再在 PICO 输入稳定后切换到 Dual-Teacher。还应确认已经等待 sim2sim 的 grounded-PD-ready 日志后才按 `a`。

### 一直显示 `tracking fallback: pico_stale`

依次检查 XRoboToolkit PC 服务、PICO 客户端连接与标定、`serve_xrobot_teleop.py`，以及是否已经按下 PICO 右手 `A`。默认输入超时为 0.25 秒。

### checkpoint 找不到

确认仓库中存在 `sim2real/config/g1/ckpts/DualTeacher/model_30000.onnx`，并且启动命令显式传入了对应的 `--task-config`。不要再依赖仓库外的 `groot_deploy_ckpts` 路径。

### 是否需要启动深度相机

当前两个 YAML 的 `input_mode` 均为 `motion`，不需要深度相机。只有将任务明确切换到 `velocity` 模式时，才需要另行启动与 YAML 中 `depth_connect` 一致的深度发布进程；不要在当前 PICO motion 部署中添加 `--input-mode velocity`。
