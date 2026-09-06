# G1 SPV5-2 deploy branch

This branch contains only the SPV5-2 policy runtime for G1:

- default checkpoint: `ckpts/0904_ckpts_74000/policy.onnx`
- additional checkpoint: `ckpts/0729_baoshou_waist_dataclean_changedr_nohandxml/policy_28000.onnx`
- desktop MuJoCo sim2sim with visualization
- G1 onboard policy inference and the C++ Unitree bridge
- remote PICO/motion references supplied by the companion `pico` branch
- optional onboard NPZ playback with `launch_deploy.sh --source motion`; the
  wireless PICO source remains the default

For one-shot host motion playback, press G1 remote A once to enter policy
control. Each host selection starts automatically when the local default
reference is ready; after completion, the same policy transitions back to its
default reference and remains active for the next selection.

See [README_zh.md](README_zh.md) for deployment commands and network settings.
