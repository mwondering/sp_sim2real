from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import yaml


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SIM2REAL_ROOT.parent
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from paths import SUPPORTED_ROBOTS
from common.udp_transport import UDPRobotHighConfig, UDPRobotLowConfig
from common.utils import DictToClass
from deploy import configure_motion_source
from runtime.motion_sources import reference_endpoint_from_env


class DeployBranchContractTests(unittest.TestCase):
    def test_requested_checkpoints_are_present(self):
        files = sorted(
            path.relative_to(REPO_ROOT).as_posix()
            for path in (REPO_ROOT / "ckpts").rglob("*")
            if path.is_file()
        )
        required = {
            "ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.json",
            "ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx",
            "ckpts/0729_baoshou_waist_dataclean_changedr_nohandxml/policy_28000.json",
            "ckpts/0729_baoshou_waist_dataclean_changedr_nohandxml/policy_28000.onnx",
            "ckpts/0903_ckpts_64000/policy.json",
            "ckpts/0903_ckpts_64000/policy.onnx",
            "ckpts/0904_ckpts_74000/policy.json",
            "ckpts/0904_ckpts_74000/policy.onnx",
        }
        self.assertEqual(required.difference(files), set())

    def test_only_g1_spv5_2_profile_is_exposed(self):
        self.assertEqual(SUPPORTED_ROBOTS, ("g1",))
        config_dir = SIM2REAL_ROOT / "config/g1"
        tracking_files = sorted(path.name for path in config_dir.glob("tracking*.yaml"))
        self.assertEqual(tracking_files, ["tracking_spv5_2.yaml"])
        tracking = yaml.safe_load((config_dir / tracking_files[0]).read_text())
        self.assertEqual(tracking["actor_profile"], "spv5_2")
        self.assertEqual(tracking["motion_source"]["type"], "vr")
        self.assertEqual(tracking["motion_source"]["vr"]["buffer_delay_s"], 0.5)
        self.assertEqual(tracking["motion_source"]["vr"]["inflight_lifetime_steps"], 25)
        self.assertNotIn("high_watermark", tracking["motion_source"]["vr"])
        self.assertNotIn("motions", tracking)
        policy = (config_dir / tracking["policy_path"]).resolve()
        self.assertEqual(
            policy,
            REPO_ROOT
            / "ckpts/0728_baoshou_waist_dataclean_changedr/policy_22000.onnx",
        )

    def test_mimiclite_style_onboard_pico_assets_are_self_contained(self):
        required = (
            SIM2REAL_ROOT / "venv/pico/pyproject.toml",
            SIM2REAL_ROOT / "venv/pico/uv.lock",
            SIM2REAL_ROOT / "install_xrobottoolkit_sdk.sh",
            SIM2REAL_ROOT / "install_xrobottoolkit_pc_service.sh",
            SIM2REAL_ROOT
            / "third_party/prebuilt/jetpack5-aarch64/xrobotservice/README.md",
            SIM2REAL_ROOT / "scripts/run_reference_server.sh",
            SIM2REAL_ROOT / "teleop/serve_xrobot_teleop.py",
            SIM2REAL_ROOT / "teleop/serve_motion_reference.py",
            SIM2REAL_ROOT / "teleop/motion_select.py",
            SIM2REAL_ROOT / "config/g1/retarget/xrobot_to_g1.json",
        )
        self.assertEqual([str(path) for path in required if not path.is_file()], [])

        teleop = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/retarget/teleop.yaml").read_text()
        )
        self.assertEqual(teleop["retarget"]["actual_human_height"], 1.80)
        alignment = teleop["retarget"]["height_alignment"]
        self.assertEqual(alignment["target_z"], 0.01)
        self.assertEqual(alignment["bootstrap_frames"], 30)
        self.assertTrue(teleop["server"]["visualize"])

    def test_deploy_launcher_exposes_onboard_pico_and_visual_motion_modes(self):
        launcher = SIM2REAL_ROOT / "scripts/launch_deploy.sh"
        result = subprocess.run(
            ["bash", str(launcher), "--help"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("--onboard-pico", result.stdout)
        self.assertIn("motion-vis", result.stdout)
        self.assertIn("--no-viewer", result.stdout)

        launcher_text = launcher.read_text()
        self.assertIn("REFERENCE_HOST=127.0.0.1", launcher_text)
        self.assertIn('REF_BUFFER_DELAY_S="${REF_BUFFER_DELAY_S:-0.0}"', launcher_text)
        self.assertIn('RETARGET_LOOKBACK_MS="${RETARGET_LOOKBACK_MS:-0.0}"', launcher_text)
        self.assertIn("xr-service", launcher_text)
        self.assertIn("reference", launcher_text)
        self.assertIn("--noprofile --norc", launcher_text)
        # send-keys remains only for delivering Ctrl-C during --stop; process
        # startup must use tmux shell-command so interactive ROS prompts cannot
        # consume the launch command.
        self.assertEqual(launcher_text.count("tmux send-keys"), 1)

    def test_xr_service_component_keeps_vendor_binary_in_foreground(self):
        launcher = SIM2REAL_ROOT / "scripts/launch_deploy.sh"
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            pico_python = temp_root / "pico/.venv/bin/python"
            pico_python.parent.mkdir(parents=True)
            pico_python.write_text("#!/bin/sh\nexit 0\n")
            pico_python.chmod(0o755)

            service_dir = temp_root / "roboticsservice"
            service_dir.mkdir()
            service_script = service_dir / "runService.sh"
            service_script.write_text("#!/bin/sh\necho wrapper-invoked\n")
            service_script.chmod(0o755)
            service_bin = service_dir / "RoboticsServiceProcess"
            service_bin.write_text(
                "#!/bin/sh\n"
                "echo foreground-service\n"
                "echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH\n"
                "exit 23\n"
            )
            service_bin.chmod(0o755)

            tool_dir = temp_root / "tools"
            tool_dir.mkdir()
            fake_ldd = tool_dir / "ldd"
            fake_ldd.write_text("#!/bin/sh\nexit 0\n")
            fake_ldd.chmod(0o755)

            env = dict(os.environ)
            env.update(
                {
                    "PATH": f"{tool_dir}:{env['PATH']}",
                    "SOURCE_MODE": "pico",
                    "PICO_RUNTIME": "onboard",
                    "PICO_PROJECT_DIR": str(temp_root / "pico"),
                    "XR_SERVICE_SCRIPT": str(service_script),
                }
            )
            result = subprocess.run(
                ["bash", str(launcher), "--sim", "--component", "xr-service"],
                env=env,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 23, result.stderr)
            self.assertIn("foreground-service", result.stdout)
            self.assertNotIn("wrapper-invoked", result.stdout)
            self.assertIn(str(service_dir / "SDK/arm64"), result.stdout)

    def test_xr_service_dependency_mismatch_fails_before_tmux_or_execution(self):
        launcher = SIM2REAL_ROOT / "scripts/launch_deploy.sh"
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            pico_python = temp_root / "pico/.venv/bin/python"
            pico_python.parent.mkdir(parents=True)
            pico_python.write_text("#!/bin/sh\nexit 0\n")
            pico_python.chmod(0o755)

            service_dir = temp_root / "roboticsservice"
            service_dir.mkdir()
            service_script = service_dir / "runService.sh"
            service_script.write_text("#!/bin/sh\nexit 0\n")
            service_script.chmod(0o755)
            service_bin = service_dir / "RoboticsServiceProcess"
            service_bin.write_text("#!/bin/sh\necho must-not-run\n")
            service_bin.chmod(0o755)

            tool_dir = temp_root / "tools"
            tool_dir.mkdir()
            fake_ldd = tool_dir / "ldd"
            fake_ldd.write_text(
                "#!/bin/sh\n"
                "echo 'libicuuc.so.70 => not found'\n"
                "exit 0\n"
            )
            fake_ldd.chmod(0o755)

            env = dict(os.environ)
            env.update(
                {
                    "PATH": f"{tool_dir}:{env['PATH']}",
                    "SOURCE_MODE": "pico",
                    "PICO_RUNTIME": "onboard",
                    "PICO_PROJECT_DIR": str(temp_root / "pico"),
                    "XR_SERVICE_SCRIPT": str(service_script),
                }
            )
            result = subprocess.run(
                ["bash", str(launcher), "--sim", "--component", "xr-service"],
                env=env,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1)
            self.assertIn("libicuuc.so.70 => not found", result.stderr)
            self.assertIn(
                "XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb",
                result.stderr,
            )
            self.assertNotIn("must-not-run", result.stdout)

    def test_ubuntu20_xr_service_installer_documents_validated_package(self):
        installer = SIM2REAL_ROOT / "install_xrobottoolkit_pc_service.sh"
        result = subprocess.run(
            ["bash", str(installer), "--help"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("Ubuntu 20.04/aarch64 runtime linker", result.stdout)
        self.assertIn(
            "XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb",
            result.stdout,
        )

    def test_reference_endpoint_environment_override(self):
        clean = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith("G1_REF_")
        }
        with patch.dict(os.environ, clean, clear=True):
            self.assertEqual(
                reference_endpoint_from_env(
                    default="tcp://127.0.0.1:28701",
                    explicit_env="G1_REF_REQ_ADDR",
                    port_env="G1_REF_REQ_PORT",
                ),
                "tcp://127.0.0.1:28701",
            )
            os.environ["G1_REF_HOST"] = "192.168.31.20"
            os.environ["G1_REF_REQ_PORT"] = "29701"
            self.assertEqual(
                reference_endpoint_from_env(
                    default="tcp://127.0.0.1:28701",
                    explicit_env="G1_REF_REQ_ADDR",
                    port_env="G1_REF_REQ_PORT",
                ),
                "tcp://192.168.31.20:29701",
            )
            os.environ["G1_REF_REQ_ADDR"] = "tcp://server-name:30701"
            self.assertEqual(
                reference_endpoint_from_env(
                    default="tcp://127.0.0.1:28701",
                    explicit_env="G1_REF_REQ_ADDR",
                    port_env="G1_REF_REQ_PORT",
                ),
                "tcp://server-name:30701",
            )

    def test_onboard_motion_override_is_explicit_and_additive(self):
        tracking = DictToClass({"motion_source": {"type": "vr", "vr": {}}})
        configure_motion_source(tracking, SimpleNamespace(motion_source="config"))
        self.assertEqual(tracking.motion_source, {"type": "vr", "vr": {}})

        with tempfile.TemporaryDirectory() as tmp_dir:
            args = SimpleNamespace(
                motion_source="motion",
                motion_root=Path(tmp_dir),
                motion_select_host="127.0.0.1",
                motion_select_port=28562,
            )
            configure_motion_source(tracking, args)

        self.assertEqual(tracking.motion_source["type"], "udp")
        self.assertEqual(
            tracking.motion_source["udp"],
            {
                "enable": True,
                "host": "127.0.0.1",
                "port": 28562,
                "motion_root": str(Path(tmp_dir).resolve()),
            },
        )

    def test_robot_udp_environment_override(self):
        config = {
            "state_bind_host": "127.0.0.1",
            "state_host": "127.0.0.1",
            "state_port": 55001,
            "cmd_host": "127.0.0.1",
            "cmd_bind_host": "127.0.0.1",
            "cmd_port": 55002,
        }
        overrides = {
            "G1_STATE_BIND_HOST": "0.0.0.0",
            "G1_STATE_HOST": "10.0.0.2",
            "G1_STATE_PORT": "56001",
            "G1_CMD_HOST": "10.0.0.3",
            "G1_CMD_BIND_HOST": "0.0.0.0",
            "G1_CMD_PORT": "56002",
        }
        with patch.dict(os.environ, overrides, clear=False):
            high = UDPRobotHighConfig.from_data(config)
            low = UDPRobotLowConfig.from_data(config)
        self.assertEqual(
            high,
            UDPRobotHighConfig("0.0.0.0", 56001, "10.0.0.3", 56002),
        )
        self.assertEqual(
            low,
            UDPRobotLowConfig("10.0.0.2", 56001, "0.0.0.0", 56002),
        )


if __name__ == "__main__":
    unittest.main()
