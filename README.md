# G1 PICO Reference Server

This branch contains only the external-host side of the split G1 deployment:
PICO/XRoboToolkit reception, G1 retargeting, reference streaming, motion
record/replay, and Viser visualization.

It intentionally contains no checkpoints, ONNX models, policy inference,
sim2sim runtime, motor control, or G1 bridge. Those belong to the `deploy`
branch.

```bash
cd sim2real
uv sync
bash install_xrobottoolkit_sdk.sh

SERVER_IP=192.168.31.20 \
REFERENCE_BIND_IP=0.0.0.0 \
VR_REQ_PORT=28701 VR_POSE_PORT=28702 VR_CTRL_PORT=28703 \
VIEWER_PORT=8080 \
bash scripts/launch_pico.sh --source pico
```

The launcher creates two local tmux windows: `xr-service` and `reference`.
In `--source motion` mode it creates `motion-select` and `reference`; motions
play once, G1 remote Up returns to the default pose, and A starts the selected
motion again.
See [README_zh.md](README_zh.md) and [sim2real/teleop/README.md](sim2real/teleop/README.md).
