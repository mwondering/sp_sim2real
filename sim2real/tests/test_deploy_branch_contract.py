from __future__ import annotations

import os
from pathlib import Path
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
            REPO_ROOT / "ckpts/0904_ckpts_74000/policy.onnx",
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
