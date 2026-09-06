# XRoboToolkit PC Service for G1 Ubuntu 20.04

Place the MimicLite JetPack 5 artifact below in this directory:

```text
XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb
```

Do not substitute the upstream generic/headless ARM64 package. That package is
built against newer ICU, GLIBC, and GLIBCXX versions than G1 Ubuntu 20.04.

Install and validate the package on G1 with:

```bash
cd sim2real
bash install_xrobottoolkit_pc_service.sh
```

If the package is not already available locally, download and verify the
pinned MimicLite archive automatically:

```bash
bash install_xrobottoolkit_pc_service.sh --download
```
