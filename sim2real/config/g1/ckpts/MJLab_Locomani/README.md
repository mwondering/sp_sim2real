# MJLab locomani policy provenance

- Source repository: `/home/lenovo/workspace/TERRAIN_LOCO/mjlab-loco`
- Source checkpoint: `logs/model_40500.pt`
  - SHA256: `5ec9c955aed2cde7862898c484607a97f6e8671db59269804a49f5b6db4323f2`
- Exported branches:
  - `logs/2026-07-26_02-58-13_lower.onnx` → `lower.onnx`
    (`3ddbc2e2abba555b5b0b0481d315fc188aa60274c8169375905104fc59382456`)
  - `logs/2026-07-26_02-58-13_upper.onnx` → `upper.onnx`
    (`927bcece752763916566d6fe7a92fb51ff66e0c2bb5ea1583db3d62a2b2f7429`)
- Verified interfaces:
  - lower: `obs [1,472]`, `depth [1,1,36,64]`,
    `lower_actor_history [1,5,100]` → `actions [1,15]`
  - upper: `obs [1,135]` → `actions [1,14]`

The ONNX outputs were numerically compared with the checkpoint actors before
integration; maximum absolute differences on deterministic random inputs were
below `2.4e-6`.

## Onboard deployment

`lower.onnx` and `upper.onnx` are architecture-independent ONNX assets. Keep
both files in this directory when copying the repository to the G1 onboard
computer; `teleop-upper-lower-locomani-real.yaml` resolves them relative to
`sim2real/config/g1/`.

The complete onboard task also requires the selected whole-body checkpoint,
the ARM64 XRoboToolkit service/binding, the separate D435i camera environment,
and a G1-native build of `g1_udp_bridge`. The intended local path is:

```text
D435i + PICO -> localhost ZMQ -> whole-body/lower/upper ONNX inference
             -> localhost UDP -> g1_udp_bridge -> Unitree DDS
```

Do not run these ONNX files directly with the generic `deploy.py`; use
`src/teleop_upper_lower_locomani.py --target real`. The full onboard command
sequence and CPU split are documented in
[`teleop-upper-lower-locomani-sim2real_zh.md`](../../../../../teleop-upper-lower-locomani-sim2real_zh.md),
and the shared setup is in
[`onboard-deployment_zh.md`](../../../../../onboard-deployment_zh.md).

The ONNX interface comparison above verifies export correctness, not onboard
latency or hardware stability. Full onboard concurrency and real-robot closed
loop behavior remain to be validated before this task can be marked onboard
verified.
