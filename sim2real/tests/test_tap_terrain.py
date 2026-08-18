from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import onnxruntime as ort
import yaml
from scipy.spatial.transform import Rotation as R


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from common.utils import DictToClass
from depth_camera import DepthRayCamera
from runtime.depth_pipeline import TAPTerrainDepthProcessor
from runtime.observation import TAPTerrainActorObservation
from runtime.policy import TAPTerrainPolicy
from deploy import Controller
from tap_terrain import (
    TASK_NAME,
    TAPTerrainController,
    TAPTerrainInputAdapter,
    _load_task_config,
)


class _FakePolicy:
    def __init__(self):
        config_path = SIM2REAL_ROOT / "config/g1/tap-terrain-policy.yaml"
        config = yaml.safe_load(config_path.read_text())
        config["_config_dir"] = str(config_path.parent)
        self.config = SimpleNamespace(**config)
        metadata = config["policy_metadata_fallback"]
        self.obs_joint_names = list(metadata["joint_names"])
        self.default_joint_pos_obs = np.asarray(
            metadata["default_joint_pos"], dtype=np.float32
        )
        self.last_action = np.linspace(-0.2, 0.2, 29, dtype=np.float32)
        self.velocity_command = np.asarray([0.4, 0.0, 0.1], dtype=np.float32)
        self.depth_image = np.full((1, 18, 32), 0.5, dtype=np.float32)
        self.controller = SimpleNamespace(
            quat=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.asarray([0.1, -0.2, 0.3], dtype=np.float32),
        )
        self._joint_pos = self.default_joint_pos_obs + 0.01
        self._joint_vel = np.linspace(-0.5, 0.5, 29, dtype=np.float32)
        self._joint_torque = np.linspace(-2.0, 2.0, 29, dtype=np.float32)

    def current_joint_pos_obs(self):
        return self._joint_pos.copy()

    def current_joint_vel_obs(self):
        return self._joint_vel.copy()

    def current_joint_torque_obs(self):
        return self._joint_torque.copy()


class _Packet:
    def __init__(self, values, metadata=None, age=0.0):
        self.values = values
        self.metadata = {} if metadata is None else metadata
        self.recv_time = time.monotonic() - age


class _Subscriber:
    def __init__(self, packet):
        self.packet = packet
        self.closed = False

    def read_latest(self):
        return self.packet

    def close(self):
        self.closed = True


class TAPTerrainTests(unittest.TestCase):
    def test_a_selection_does_not_enable_before_first_policy_target(self):
        sent_enables = []
        selected_policy = SimpleNamespace(fade_in=lambda: None)
        controller = SimpleNamespace(
            btn_rise={"stop": False, "A": True},
            cmd_q=np.zeros(29, dtype=np.float32),
            cmd_qd=np.zeros(29, dtype=np.float32),
            cmd_kp=np.zeros(29, dtype=np.float32),
            cmd_kd=np.zeros(29, dtype=np.float32),
            cmd_enable=0,
            init_qpos=np.ones(29, dtype=np.float32),
            kps=np.ones(29, dtype=np.float32),
            kds=np.ones(29, dtype=np.float32),
            policies={"tracking": selected_policy},
            process_state=lambda **kwargs: True,
        )
        controller.send_cmd = lambda: sent_enables.append(controller.cmd_enable)

        Controller.default_qpos_state(controller, enable_immediately=False)

        self.assertIs(controller.current_policy, selected_policy)
        self.assertEqual(controller.cmd_enable, 0)
        self.assertEqual(sent_enables, [0])

    def test_tap_handoff_waits_for_depth_then_sends_first_action_enabled(self):
        policy = object.__new__(TAPTerrainPolicy)
        calls = {"compute": 0, "state": 0, "depth": 0}
        policy.set_external_inputs = lambda velocity, depth: None
        policy.update_obs = lambda: None
        policy.policy_observation_copy = lambda: np.zeros(7484, dtype=np.float32)

        def compute_action():
            calls["compute"] += 1
            return np.zeros(29, dtype=np.float32)

        policy.compute_action = compute_action
        policy.post_step = lambda: None

        controller = object.__new__(TAPTerrainController)
        controller.policies = {"tracking": policy}
        controller._start_policy_recorder = lambda: None
        controller.dof_size = 29
        controller.default_qpos = np.zeros(29, dtype=np.float32)
        controller.kps = np.ones(29, dtype=np.float32)
        controller.kds = np.ones(29, dtype=np.float32)
        controller.cmd_q = np.zeros(29, dtype=np.float32)
        controller.cmd_qd = np.zeros(29, dtype=np.float32)
        controller.cmd_kp = np.zeros(29, dtype=np.float32)
        controller.cmd_kd = np.zeros(29, dtype=np.float32)
        controller.cmd_enable = 0
        controller.history_warmup_steps = 0
        controller._depth_ready_once = False
        controller._last_wait_report = 0.0
        controller.btn_rise = {"stop": False}
        controller.sticks = {}
        controller.recorder = None
        controller.policy_step = 0
        sent_enables = []
        controller.send_cmd = lambda: sent_enables.append(controller.cmd_enable)

        def process_state(**kwargs):
            calls["state"] += 1
            controller.btn_rise["stop"] = calls["state"] == 3
            return True

        def read_depth():
            calls["depth"] += 1
            if calls["depth"] == 1:
                return None, "depth_not_received"
            return np.full((1, 18, 32), 0.5, dtype=np.float32), "ready"

        controller.process_state = process_state
        controller.tap_inputs = SimpleNamespace(
            read_depth=read_depth,
            velocity_command=lambda sticks: np.asarray([0.5, 0.0, 0.0], dtype=np.float32),
        )

        controller.run_tap_terrain()

        self.assertEqual(sent_enables, [0, 1])
        self.assertEqual(calls["compute"], 1)

    def test_observation_matches_exported_7484_layout(self):
        observation = TAPTerrainActorObservation(_FakePolicy())
        observation.update()
        value = observation.compute()
        self.assertEqual(value.shape, (7484,))
        self.assertTrue(np.isfinite(value).all())

        # The 805-D prefix starts with the latest five frames of each term.
        latest_joint_pos = value[: 5 * 29]
        history_start = 805
        full_joint_pos = value[history_start : history_start + 50 * 29]
        np.testing.assert_array_equal(
            latest_joint_pos, full_joint_pos.reshape(50, 29)[-5:].reshape(-1)
        )
        # TAP training resets the older 49 samples to zero and inserts the
        # current sample at the newest position before the first action.
        joint_pos_frames = full_joint_pos.reshape(50, 29)
        np.testing.assert_array_equal(joint_pos_frames[:-1], 0.0)
        np.testing.assert_allclose(joint_pos_frames[-1], 0.01, atol=1.0e-7)
        np.testing.assert_allclose(value[6905:6908], [0.4, 0.0, 0.1])
        np.testing.assert_array_equal(value[6908:], 0.5)

    def test_depth_processor_matches_training_crop_blur_and_range(self):
        processor = TAPTerrainDepthProcessor()
        depth, stats = processor.process(
            np.full((36, 64), 0.6, dtype=np.float32)
        )
        self.assertEqual(depth.shape, (1, 18, 32))
        self.assertEqual(depth.dtype, np.float32)
        np.testing.assert_allclose(depth, 0.5, atol=1.0e-6)
        self.assertEqual(stats["invalid_fraction"], 0.0)

        invalid, stats = processor.process(np.zeros((360, 640), dtype=np.float32))
        np.testing.assert_array_equal(invalid, 1.0)
        self.assertEqual(stats["raw_invalid_fraction"], 1.0)

    def test_depth_camera_can_publish_tap_policy_shape(self):
        xml_path = (
            SIM2REAL_ROOT
            / "config/g1/assets/g1_teleop_upper_lower_locomani.xml"
        )
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        data.qpos[:7] = [-1.0, -10.0, 0.76, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)
        camera = DepthRayCamera(
            model,
            data,
            site_name="tap_terrain_depth_camera",
            depth_processor=TAPTerrainDepthProcessor(),
        )
        training_rotation = R.from_quat(
            [
                0.680026004439802,
                0.18380950076459376,
                -0.18221241872180804,
                -0.6859863957673342,
            ],
            scalar_first=True,
        )
        site_rotation = data.site_xmat[camera.site_id].reshape(3, 3)
        # Training/MuJoCo camera axes are (+X right, +Y up, -Z forward),
        # while DepthRayCamera emits (+X forward, -Y right, -Z down).
        np.testing.assert_allclose(
            site_rotation @ [1.0, 0.0, 0.0],
            training_rotation.apply([0.0, 0.0, -1.0]),
            atol=1.0e-7,
        )
        np.testing.assert_allclose(
            site_rotation @ [0.0, -1.0, 0.0],
            training_rotation.apply([1.0, 0.0, 0.0]),
            atol=1.0e-7,
        )
        depth = camera.capture()
        self.assertEqual(depth.shape, (1, 18, 32))
        self.assertTrue(np.isfinite(depth).all())
        self.assertLess(float(depth.min()), 1.0)
        self.assertGreater(float(depth.std()), 0.0)

    def test_input_adapter_rejects_stale_depth_and_builds_fixed_command(self):
        task = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/tap-terrain.yaml").read_text()
        )
        subscriber = _Subscriber(
            _Packet(np.full((1, 18, 32), 0.5, dtype=np.float32))
        )
        adapter = TAPTerrainInputAdapter(
            target="sim",
            camera_config=task["camera_process"],
            velocity_config=task["velocity_command"],
            subscriber=subscriber,
        )
        depth, reason = adapter.read_depth()
        self.assertEqual(reason, "ready")
        np.testing.assert_array_equal(depth, 0.5)
        np.testing.assert_array_equal(
            adapter.velocity_command({}), [0.0, 0.0, 0.0]
        )
        np.testing.assert_array_equal(
            adapter.velocity_command({"ly": 1.0, "lx": 1.0, "rx": 1.0}),
            [1.0, 0.5, 0.5],
        )
        np.testing.assert_array_equal(
            adapter.velocity_command({"ly": -1.0, "lx": -1.0, "rx": -1.0}),
            [-1.0, -0.5, -0.5],
        )

        fixed_adapter = TAPTerrainInputAdapter(
            target="sim",
            camera_config=task["camera_process"],
            velocity_config=task["velocity_command"],
            velocity_override=[0.2, 0.0, 0.0],
            subscriber=_Subscriber(subscriber.packet),
        )
        np.testing.assert_allclose(
            fixed_adapter.velocity_command({"ly": -1.0}),
            [0.2, 0.0, 0.0],
            atol=1.0e-7,
        )
        fixed_adapter.close()

        subscriber.packet = _Packet(
            np.full((1, 18, 32), 0.5, dtype=np.float32), age=1.0
        )
        depth, reason = adapter.read_depth()
        self.assertIsNone(depth)
        self.assertIn("depth_stale", reason)
        adapter.close()
        self.assertTrue(subscriber.closed)

    def test_task_and_real_onnx_contract_resolve(self):
        task_path = SIM2REAL_ROOT / "config/g1/tap-terrain.yaml"
        task = _load_task_config(task_path)
        self.assertEqual(task["task_name"], TASK_NAME)
        self.assertEqual(task["history_warmup_steps"], 0)
        self.assertEqual(task["velocity_command"]["source"], "sticks")
        self.assertEqual(task["velocity_command"]["fixed"], [0.0, 0.0, 0.0])
        self.assertFalse(task["velocity_command"]["forward_only"])
        real_task = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/tap-terrain-real.yaml").read_text()
        )
        self.assertEqual(real_task["history_warmup_steps"], 0)
        bridge_path = (task_path.parent / task["sim_bridge_config"]).resolve()
        bridge = yaml.safe_load(bridge_path.read_text())
        self.assertTrue(bridge_path.is_file())
        self.assertEqual(len(bridge["policy_joint_names"]), 29)
        self.assertEqual(len(bridge["mujoco_joint_names"]), 29)
        self.assertEqual(len(bridge["home_q"]), 29)
        self.assertEqual(bridge["root_qpos_control"][:3], [-1.0, -10.0, 0.76])
        self.assertTrue(bridge["sim_state_stream"]["enabled"])
        self.assertFalse(bridge["depth_debug"]["enabled"])
        self.assertTrue(bridge["keyboard"]["motion_enabled"])
        self.assertEqual(
            (
                bridge["keyboard"]["start_key"],
                bridge["keyboard"]["policy_key"],
                bridge["keyboard"]["stop_key"],
            ),
            ("r", "p", "x"),
        )
        policy_path = (task_path.parent / task["tracking_config"]).resolve()
        policy = yaml.safe_load(policy_path.read_text())
        onnx_path = (policy_path.parent / policy["policy_path"]).resolve()
        self.assertTrue(onnx_path.is_file(), onnx_path)
        self.assertTrue(onnx_path.with_suffix(".json").is_file())
        session = ort.InferenceSession(
            str(onnx_path), providers=["CPUExecutionProvider"]
        )
        self.assertEqual(
            [(item.name, item.shape) for item in session.get_inputs()],
            [("spv5_2_terrain_observation", [1, 7484])],
        )
        self.assertEqual(
            [(item.name, item.shape) for item in session.get_outputs()],
            [("action", [1, 29])],
        )

    def test_policy_wrapper_runs_the_exported_model(self):
        policy_path = SIM2REAL_ROOT / "config/g1/tap-terrain-policy.yaml"
        policy_dict = yaml.safe_load(policy_path.read_text())
        policy_dict["_config_dir"] = str(policy_path.parent)
        policy_dict["_config_path"] = str(policy_path)
        controller_dict = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/controller.yaml").read_text()
        )
        controller_config = DictToClass(controller_dict)
        joint_count = len(controller_config.policy_joint_names)
        controller = SimpleNamespace(
            config=controller_config,
            dof_size=joint_count,
            qj=np.asarray(controller_config.default_qpos, dtype=np.float32),
            dqj=np.zeros(joint_count, dtype=np.float32),
            tau_latest=np.zeros(joint_count, dtype=np.float32),
            quat=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.zeros(3, dtype=np.float32),
        )
        policy = TAPTerrainPolicy(
            "tap_terrain", DictToClass(policy_dict), controller
        )
        policy.set_external_inputs(
            np.asarray([0.5, 0.0, 0.0], dtype=np.float32),
            np.full((1, 18, 32), 0.5, dtype=np.float32),
        )
        policy.update_obs()
        self.assertEqual(policy.policy_observation_copy().shape, (7484,))
        action = policy.compute_action()
        self.assertEqual(action.shape, (29,))
        self.assertTrue(np.isfinite(action).all())


if __name__ == "__main__":
    unittest.main()
