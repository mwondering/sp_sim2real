from __future__ import annotations

from dataclasses import replace
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import yaml


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from depth_camera import DepthRayCamera, _resize_bilinear_align_corners_false
from runtime.d435i_source import D435iSource
from runtime.depth_pipeline import RealDepthProcessor
from runtime.depth_overlay import DepthPointCloudOverlay
from runtime.dual_locomani import (
    ALL_JOINT_NAMES,
    DEFAULT_JOINT_POS,
    DualLocomaniRuntime,
    ObservationBuilder,
    PicoDualReferenceBuilder,
)
from runtime.shared_pico import PicoSnapshot
from teleop_upper_lower_locomani import TeleopUpperLowerLocomaniController


class TeleopUpperLowerLocomaniTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml_path = (
            SIM2REAL_ROOT
            / "config/g1/assets/g1_teleop_upper_lower_locomani.xml"
        )

    def _snapshot(self, *, buttons=None, sticks=None, active=True, age=0.0):
        now = time.monotonic()
        return PicoSnapshot(
            seq=1,
            timestamp=now,
            pose_timestamp=now - age,
            control_timestamp=now - age,
            joint_names=ALL_JOINT_NAMES,
            joint_pos=DEFAULT_JOINT_POS.copy(),
            root_pos=np.array([0.0, 0.0, 0.76], dtype=np.float32),
            root_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            buttons={} if buttons is None else buttons,
            sticks={} if sticks is None else sticks,
            active=active,
        )

    def test_scene_contains_training_terrain_and_camera_mount(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        site_id = model.site("depth_camera").id
        self.assertGreaterEqual(site_id, 0)
        np.testing.assert_allclose(
            model.site_pos[site_id],
            [0.12546158, 0.0197, -0.05952004],
            atol=1.0e-6,
        )
        np.testing.assert_allclose(
            model.site_quat[site_id],
            [0.8660254, 0.0, 0.5, 0.0],
            atol=1.0e-6,
        )
        self.assertGreaterEqual(model.geom("stairs_platform").id, 0)
        self.assertGreaterEqual(model.geom("slope_up").id, 0)

    def test_depth_camera_produces_policy_shape(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        data = mujoco.MjData(model)
        data.qpos[:7] = [-1.0, 0.0, 0.76, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)
        depth = DepthRayCamera(model, data).capture()
        self.assertEqual(depth.shape, (1, 36, 64))
        self.assertEqual(depth.dtype, np.float32)
        self.assertTrue(np.isfinite(depth).all())

    def test_mujoco_depth_overlay_reconstructs_colored_point_samples(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        data = mujoco.MjData(model)
        data.qpos[:7] = [-1.0, 0.0, 0.76, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)
        overlay = object.__new__(DepthPointCloudOverlay)
        overlay.model = model
        overlay.data = data
        overlay.site_id = model.site("depth_camera").id
        overlay.stride = 4
        overlay.sample_rows = np.arange(0, 36, 4, dtype=np.int32)
        overlay.sample_cols = np.arange(0, 64, 4, dtype=np.int32)
        overlay.local_directions = (
            DepthPointCloudOverlay._output_pixel_directions()
        )
        points, values = overlay.world_points(
            np.full((1, 36, 64), 0.5, dtype=np.float32)
        )
        self.assertEqual(points.shape, (144, 3))
        np.testing.assert_allclose(values, 0.5)
        self.assertTrue(np.isfinite(points).all())

    def test_bilinear_resize_keeps_constant_image(self):
        image = np.full((26, 44), 0.37, dtype=np.float32)
        resized = _resize_bilinear_align_corners_false(image, (36, 64))
        np.testing.assert_allclose(resized, 0.37, atol=1.0e-6)

    def test_d435i_preprocess_matches_policy_shape_and_normalization(self):
        processor = RealDepthProcessor()
        processed, stats = processor.process(
            np.full((360, 640), 1.0, dtype=np.float32)
        )
        self.assertEqual(processed.shape, (1, 36, 64))
        self.assertEqual(processed.dtype, np.float32)
        np.testing.assert_allclose(processed, 0.5, atol=1.0e-6)
        self.assertEqual(stats["invalid_fraction"], 0.0)

    def test_d435i_preprocess_keeps_large_invalid_region_sentinel(self):
        processor = RealDepthProcessor(edge_fill_left_columns=0)
        raw = np.ones((360, 640), dtype=np.float32)
        raw[:, 200:440] = 0.0
        processed, stats = processor.process(raw)
        self.assertGreater(stats["invalid_fraction"], 0.2)
        self.assertTrue(np.any(processed == -1.0))

    def test_stairs_forces_forward_only_but_allows_neutral_stop(self):
        builder = PicoDualReferenceBuilder(
            kinematics_xml=self.xml_path,
            controller_joint_names=ALL_JOINT_NAMES,
            terrain_class=2,
        )
        moving = builder.build(
            self._snapshot(sticks={"ly": 0.2, "lx": 1.0, "rx": 1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(moving.twist, [0.3, 0.0, 0.0, 0.65])
        stopped = builder.build(
            self._snapshot(sticks={"ly": 0.0, "lx": 1.0, "rx": 1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(stopped.twist, [0.0, 0.0, 0.0, 0.65])
        reverse = builder.build(
            self._snapshot(sticks={"ly": -1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(reverse.twist, [0.0, 0.0, 0.0, 0.65])

    def test_stale_pico_zeroes_velocity_and_invalidates_reference(self):
        builder = PicoDualReferenceBuilder(
            kinematics_xml=self.xml_path,
            controller_joint_names=ALL_JOINT_NAMES,
            terrain_class=1,
            pose_timeout_s=0.25,
        )
        reference = builder.build(
            self._snapshot(sticks={"ly": 1.0}, age=1.0),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        self.assertFalse(reference.pico_valid)
        np.testing.assert_allclose(reference.twist[:3], 0.0)

    def test_pico_left_x_requests_software_stop_only_when_control_is_fresh(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.pico_stop_enabled = True
        controller.pico_stop_button = "left_key_one"
        controller.pico_stop_timeout_s = 0.25
        now = time.monotonic()
        fresh = self._snapshot(buttons={"left_key_one": True})
        self.assertTrue(
            controller._pico_software_stop_requested(fresh, now=now)
        )
        stale = replace(fresh, control_timestamp=now - 1.0)
        self.assertFalse(
            controller._pico_software_stop_requested(stale, now=now)
        )
        released = replace(fresh, buttons={"left_key_one": False})
        self.assertFalse(
            controller._pico_software_stop_requested(released, now=now)
        )

    def test_remote_depth_freshness_uses_server_receive_clock(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.depth_timeout_s = 0.25
        controller.max_invalid_fraction = 0.6
        now = time.monotonic()
        packet = SimpleNamespace(
            recv_time=now - 0.02,
            values=np.full((1, 36, 64), 0.5, dtype=np.float32),
            metadata={
                # A remote monotonic timestamp has an unrelated epoch and must
                # not participate in the policy server's freshness decision.
                "capture_monotonic": now - 1_000_000.0,
                "invalid_fraction": 0.0,
                "invalid_value": -1.0,
            },
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: packet)
        np.testing.assert_allclose(controller._depth(), 0.5)

    def test_remote_depth_rejects_stale_or_future_receive_timestamp(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.depth_timeout_s = 0.25
        controller.max_invalid_fraction = 0.6
        now = time.monotonic()
        values = np.full((1, 36, 64), 0.5, dtype=np.float32)
        stale = SimpleNamespace(
            recv_time=now - 1.0,
            values=values,
            metadata={"capture_monotonic": now, "invalid_fraction": 0.0},
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: stale)
        self.assertIsNone(controller._depth())

        future = SimpleNamespace(
            recv_time=now + 1.0,
            values=values,
            metadata={"capture_monotonic": now, "invalid_fraction": 0.0},
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: future)
        self.assertIsNone(controller._depth())

    def test_worker_metadata_enforces_profile_serial_and_fov(self):
        source = object.__new__(D435iSource)
        source.width = 640
        source.height = 360
        source.fps = 30
        source.serial_number = "1234"
        source.expected_fov = (89.04, 57.9)
        source.fov_tolerance_deg = 6.0
        source.profile = None
        source.depth_scale = 0.001
        source.fov_deg = None
        source._worker_device_serial = None
        source._worker_device_name = None
        metadata = {
            "protocol_version": 1,
            "device_name": "Intel RealSense D435I",
            "serial_number": "1234",
            "depth_scale": 0.001,
            "width": 640,
            "height": 360,
            "fps": 30,
            "fov_x_deg": 88.0,
            "fov_y_deg": 58.0,
        }
        source._apply_worker_metadata(metadata)
        self.assertEqual(source.device_serial, "1234")
        self.assertEqual(source.fov_deg, (88.0, 58.0))
        with self.assertRaisesRegex(
            RuntimeError, "视场角与训练配置差异过大"
        ):
            source._apply_worker_metadata(
                {**metadata, "fov_x_deg": 120.0}
            )
        with self.assertRaisesRegex(RuntimeError, "unexpected device"):
            source._apply_worker_metadata(
                {**metadata, "serial_number": "5678"}
            )
        with self.assertRaisesRegex(RuntimeError, "Expected.*D435i"):
            source._apply_worker_metadata(
                {**metadata, "device_name": "Intel RealSense D455"}
            )

    def test_copied_dual_onnx_pair_runs_with_exact_contract(self):
        controller_names = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/controller.yaml").read_text()
        )["policy_joint_names"]
        runtime = DualLocomaniRuntime(
            lower_onnx=SIM2REAL_ROOT
            / "config/g1/ckpts/MJLab_Locomani/lower.onnx",
            upper_onnx=SIM2REAL_ROOT
            / "config/g1/ckpts/MJLab_Locomani/upper.onnx",
            kinematics_xml=self.xml_path,
            controller_joint_names=controller_names,
            terrain_class=1,
        )
        canonical_index = {name: i for i, name in enumerate(ALL_JOINT_NAMES)}
        controller = SimpleNamespace(
            qj=np.asarray(
                [DEFAULT_JOINT_POS[canonical_index[name]] for name in controller_names],
                dtype=np.float32,
            ),
            dqj=np.zeros(29, dtype=np.float32),
            quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.zeros(3, dtype=np.float32),
        )
        command, reference = runtime.step(
            controller=controller,
            snapshot=self._snapshot(sticks={"ly": 0.5}),
            depth=np.full((1, 36, 64), 0.5, dtype=np.float32),
        )
        self.assertTrue(reference.pico_valid)
        self.assertEqual(command.q_des.shape, (29,))
        self.assertTrue(np.isfinite(command.q_des).all())
        # Inference alone must not claim that its candidate action was applied.
        np.testing.assert_allclose(runtime.observation.last_action, 0.0)
        runtime.commit_applied_target(command.q_des)
        self.assertFalse(np.allclose(runtime.observation.last_action, 0.0))

    def test_real_config_records_confirmed_camera_mount_and_separate_source(self):
        config = yaml.safe_load(
            (
                SIM2REAL_ROOT
                / "config/g1/teleop-upper-lower-locomani-real.yaml"
            ).read_text()
        )
        self.assertEqual(config["target"], "real")
        self.assertEqual(config["camera_process"]["source"], "d435i")
        self.assertEqual(config["switch"]["blend_duration_s"], 1.0)
        self.assertEqual(
            config["pico_software_stop"]["button"], "left_key_one"
        )
        hardware = config["camera_process"]["hardware"]
        self.assertEqual(hardware["mount_body"], "pelvis")
        self.assertAlmostEqual(float(hardware["pitch_down_deg"]), 60.0)

    def test_distributed_profiles_preserve_policy_contract_and_split_hosts(self):
        config_dir = SIM2REAL_ROOT / "config/g1"
        local_controller = yaml.safe_load(
            (config_dir / "controller.yaml").read_text()
        )
        distributed_controller = yaml.safe_load(
            (config_dir / "controller-distributed.yaml").read_text()
        )
        for key, value in local_controller.items():
            if key != "udp":
                self.assertEqual(distributed_controller[key], value)
        self.assertEqual(
            distributed_controller["udp"]["state_bind_host"], "10.42.0.1"
        )
        self.assertEqual(
            distributed_controller["udp"]["cmd_host"], "10.42.0.2"
        )

        local_task = yaml.safe_load(
            (
                config_dir
                / "teleop-upper-lower-locomani-real.yaml"
            ).read_text()
        )
        distributed_task = yaml.safe_load(
            (
                config_dir
                / "teleop-upper-lower-locomani-real-distributed.yaml"
            ).read_text()
        )
        for key, value in local_task.items():
            if key != "camera_process":
                self.assertEqual(distributed_task[key], value)
        for key, value in local_task["camera_process"].items():
            if key not in ("depth_bind", "depth_connect"):
                self.assertEqual(
                    distributed_task["camera_process"][key], value
                )
        self.assertEqual(
            distributed_task["camera_process"]["depth_connect"],
            "tcp://10.42.0.2:28811",
        )

        retarget = yaml.safe_load(
            (config_dir / "retarget/teleop-server.yaml").read_text()
        )
        for field in ("req_bind_addr", "rep_bind_addr", "ctrl_bind_addr"):
            self.assertTrue(
                retarget["server"][field].startswith("tcp://127.0.0.1:")
            )

        bridge_config_dir = SIM2REAL_ROOT.parent / "g1_sim2real/config"
        local_bridge = yaml.safe_load(
            (
                bridge_config_dir
                / "g1_bridge_teleop_upper_lower_locomani.yaml"
            ).read_text()
        )
        bridge = yaml.safe_load(
            (
                bridge_config_dir
                / "g1_bridge_teleop_upper_lower_locomani_distributed.yaml"
            ).read_text()
        )
        for key, value in local_bridge.items():
            if key != "udp":
                self.assertEqual(bridge[key], value)
        self.assertEqual(bridge["udp"]["state_host"], "10.42.0.1")
        self.assertEqual(bridge["udp"]["cmd_bind_host"], "10.42.0.2")
        self.assertEqual(bridge["udp"]["cmd_allowed_host"], "10.42.0.1")
        self.assertTrue(bridge["safety"]["startup_damping"])
        self.assertEqual(bridge["safety"]["damping_publish_hz"], 50.0)

    def test_spv5_2_task_option_resolves_inside_current_repository(self):
        config_path = SIM2REAL_ROOT / "config/g1/tracking_spv5_2.yaml"
        config = yaml.safe_load(config_path.read_text())
        policy = (config_path.parent / config["policy_path"]).resolve()
        self.assertTrue(policy.is_file(), policy)
        self.assertTrue(policy.with_suffix(".json").is_file())

    def test_original_routes_remain_opt_in(self):
        original_bridge = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/bridge.yaml").read_text()
        )
        original_tracking = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/tracking.yaml").read_text()
        )
        self.assertNotIn("sim_state_stream", original_bridge)
        self.assertEqual(original_tracking["motion_source"]["type"], "udp")
        original_real_bridge = yaml.safe_load(
            (
                SIM2REAL_ROOT.parent / "g1_sim2real/config/g1_bridge.yaml"
            ).read_text()
        )
        self.assertNotIn("safety", original_real_bridge)


if __name__ == "__main__":
    unittest.main()
