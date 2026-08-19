# TAP 机载启动命令

## 无遥操：仅 terrain

### 终端 1：本地外接 D435i

```bash
cd /home/unitree/g1-camera-stream
taskset -c 0-1 .venv/bin/python depth_camera_sender.py --config camera.yaml
```

### 终端 2：G1 bridge

```bash
export TAP_REPO="/home/unitree/motion_tracking_sim2real_self"
cd "${TAP_REPO}/g1_sim2real"
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_tap_onboard \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
taskset -c 2-3 bash scripts/run_bridge.sh
```

### 终端 3：terrain policy

```bash
export TAP_REPO="/home/unitree/motion_tracking_sim2real_self"
cd "${TAP_REPO}/sim2real"
taskset -c 4-7 uv run src/tap_terrain.py \
  --target real \
  --terrain-only
```

## 有遥操：terrain 与 teleop 切换

### 终端 1：本地外接 D435i

```bash
cd /home/unitree/g1-camera-stream
taskset -c 0-1 .venv/bin/python depth_camera_sender.py --config camera.yaml
```

### 终端 2：XRoboToolkit PC Service

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

### 终端 3：PICO 重定向

```bash
export TAP_REPO="/home/unitree/motion_tracking_sim2real_self"
cd "${TAP_REPO}/sim2real"
bash scripts/run_pico_server.sh
```

### 终端 4：G1 bridge

```bash
export TAP_REPO="/home/unitree/motion_tracking_sim2real_self"
cd "${TAP_REPO}/g1_sim2real"
G1_NET=eth0 \
G1_BRIDGE_BUILD_DIR=build_tap_onboard \
G1_BRIDGE_CONFIG=config/g1_bridge.yaml \
taskset -c 2-3 bash scripts/run_bridge.sh
```

### 终端 5：terrain + teleop policy

```bash
export TAP_REPO="/home/unitree/motion_tracking_sim2real_self"
cd "${TAP_REPO}/sim2real"
taskset -c 4-7 uv run src/tap_terrain.py --target real
```
