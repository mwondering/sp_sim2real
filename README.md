# G1 SPV5-2 deployment

This branch contains the G1 SPV5-2 sim2sim/sim2real runtime and now supports
the MimicLite-style onboard PICO layout:

- policy inference and the C++ Unitree bridge run on the G1;
- XRoboToolkit PC Service and PICO-to-G1 retargeting can run on the same G1;
- the retarget process includes a LAN-accessible MJViser view;
- policy/retarget and policy/bridge traffic stays on `127.0.0.1`;
- the existing external-PICO and direct onboard-NPZ modes remain available.

The default policy is
`ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx`.

Quick start after completing the ARM64 setup in
[README_zh.md](README_zh.md):

```bash
cd sim2real
VIEWER_URL_HOST=<g1-ip> bash scripts/launch_deploy.sh --real --onboard-pico
```

Open `http://<g1-ip>:8080` to inspect the live human skeleton and retargeted
G1 motion before enabling policy control.

For visualized NPZ playback:

```bash
bash scripts/launch_deploy.sh --real --source motion-vis
```

For the lowest-overhead onboard NPZ path, keep using
`--source motion`. For an external PICO host, keep using
`REFERENCE_HOST=<host-ip> ... --source pico`.

See [README_zh.md](README_zh.md) for setup, process layout, controls, ports,
latency settings, and Jetson notes.
