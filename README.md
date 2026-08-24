# G1 SPV5-2 deploy branch

This branch contains only the SPV5-2 policy runtime for G1:

- checkpoint: `ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx`
- desktop MuJoCo sim2sim with visualization
- G1 onboard policy inference and the C++ Unitree bridge
- remote PICO/motion references supplied by the companion `pico` branch

For one-shot host motion playback, press G1 remote A once to enter policy
control. Each host selection starts automatically when the local default
reference is ready; after completion, the same policy transitions back to its
default reference and remains active for the next selection.

See [README_zh.md](README_zh.md) for deployment commands and network settings.
