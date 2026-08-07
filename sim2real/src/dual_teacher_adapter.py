from __future__ import annotations

import argparse
import time
import traceback
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Sequence

import numpy as np
import onnxruntime as ort
import yaml
from scipy.spatial.transform import Rotation as R

from deploy import Controller, get_config
from paths import controller_config_path
from runtime.depth_pipeline import RealDepthProcessor
from runtime.kinematics import joint_velocity_from_positions
from runtime.shared_pico import PicoSnapshot
from runtime.shared_pico import PicoFrameStore
from runtime.zmq_stream import ArraySubscriber


InputMode = Literal["motion", "velocity"]
ControlSource = Literal["tracking", "dual"]


SIM2REAL_ROOT = Path(__file__).resolve().parent.parent


CANONICAL_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


DEFAULT_JOINT_POS = np.array(
    [
        -0.28,
        0.0,
        0.0,
        0.5,
        -0.23,
        0.0,
        -0.28,
        0.0,
        0.0,
        0.5,
        -0.23,
        0.0,
        0.0,
        0.0,
        0.0,
        0.35,
        0.16,
        0.0,
        0.87,
        0.0,
        0.0,
        0.0,
        0.35,
        -0.16,
        0.0,
        0.87,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


ACTION_SCALE = np.array([0.5] * 15 + [1.0] * 14, dtype=np.float32)


def _joint_gains(names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    stiffness = []
    damping = []
    for name in names:
        if "wrist_pitch" in name or "wrist_yaw" in name:
            kp, kd = 8.611032447370201, 0.548195351665136
        elif (
            "shoulder" in name
            or "elbow" in name
            or "wrist_roll" in name
        ):
            kp, kd = 14.25062309787429, 0.907222843292423
        elif "hip_yaw" in name or name == "waist_yaw_joint":
            kp, kd = 40.17923847137318, 2.5578897650279457
        elif "hip_pitch" in name or "hip_roll" in name or "knee" in name:
            kp, kd = 99.09842777666113, 6.3088018534966395
        elif "ankle" in name or name in {
            "waist_roll_joint",
            "waist_pitch_joint",
        }:
            kp, kd = 28.50124619574858, 1.814445686584846
        else:
            raise ValueError(f"No dual-encoder gain contract for joint {name!r}")
        stiffness.append(kp)
        damping.append(kd)
    return (
        np.asarray(stiffness, dtype=np.float32),
        np.asarray(damping, dtype=np.float32),
    )


JOINT_KP, JOINT_KD = _joint_gains(CANONICAL_JOINT_NAMES)


@dataclass(frozen=True)
class ControlCommand:
    q_des: np.ndarray
    kp: np.ndarray
    kd: np.ndarray


@dataclass(frozen=True)
class DualEncoderStepResult:
    command: ControlCommand | None
    ready: bool
    reason: str
    input_mode: InputMode


def _name_indices(
    source_names: Sequence[str], target_names: Sequence[str]
) -> np.ndarray:
    source = tuple(str(name) for name in source_names)
    target = tuple(str(name) for name in target_names)
    if len(set(source)) != len(source):
        raise ValueError("Source joint names contain duplicates")
    if len(set(target)) != len(target):
        raise ValueError("Target joint names contain duplicates")
    source_index = {name: index for index, name in enumerate(source)}
    missing = [name for name in target if name not in source_index]
    if missing:
        raise ValueError(f"Joint mapping is missing names: {missing}")
    return np.asarray([source_index[name] for name in target], dtype=np.int32)


def reference_joint_velocity(
    joint_pos: np.ndarray, index: int, fps: float
) -> np.ndarray:
    """Return HEFT/Data10k joint velocity for one reference frame.

    ``fk_backend_compare`` computes a length-aware central difference and then
    applies a replicate-padded five-point average.  The shared deployment
    helper implements the same operation for a complete unpadded sequence.
    """

    values = np.asarray(joint_pos, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != len(CANONICAL_JOINT_NAMES):
        raise ValueError(
            "Reference joint positions must have shape "
            f"[T,{len(CANONICAL_JOINT_NAMES)}], got {values.shape}"
        )
    if values.shape[0] == 0:
        raise ValueError("Reference joint position sequence is empty")
    frame = int(index)
    if frame < 0 or frame >= values.shape[0]:
        raise IndexError(
            f"Reference frame {frame} is outside [0,{values.shape[0]})"
        )
    return joint_velocity_from_positions(values, float(fps))[frame].astype(
        np.float32, copy=True
    )


def _resolve(config_path: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()


def _load_task_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Task config must be a mapping: {path}")
    if config.get("task_name") != "dual-teacher":
        raise ValueError(
            f"Task config {path} has task_name={config.get('task_name')!r}, "
            "expected 'dual-teacher'"
        )
    return config


def _append_with_backfill(
    history: deque[np.ndarray], value: np.ndarray
) -> None:
    sample = np.asarray(value, dtype=np.float32).copy()
    if not history:
        history.extend(sample.copy() for _ in range(history.maxlen or 1))
    else:
        history.append(sample)


class DualEncoderObservationBuilder:
    HISTORY_LENGTH = 5

    def __init__(self) -> None:
        self._histories = {
            "gravity": deque(maxlen=self.HISTORY_LENGTH),
            "ang_vel": deque(maxlen=self.HISTORY_LENGTH),
            "joint_pos": deque(maxlen=self.HISTORY_LENGTH),
            "joint_vel": deque(maxlen=self.HISTORY_LENGTH),
            "action": deque(maxlen=self.HISTORY_LENGTH),
        }
        self.last_applied_action = np.zeros(29, dtype=np.float32)

    def reset(self) -> None:
        for history in self._histories.values():
            history.clear()
        self.last_applied_action.fill(0.0)

    def observe(
        self,
        *,
        projected_gravity: np.ndarray,
        base_ang_vel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        q = np.asarray(joint_pos, dtype=np.float32).reshape(29)
        terms = {
            "gravity": np.asarray(projected_gravity, dtype=np.float32).reshape(3),
            "ang_vel": np.asarray(base_ang_vel, dtype=np.float32).reshape(3),
            "joint_pos": q - DEFAULT_JOINT_POS,
            "joint_vel": np.asarray(joint_vel, dtype=np.float32).reshape(29),
            "action": self.last_applied_action,
        }
        for name, value in terms.items():
            if not np.isfinite(value).all():
                raise ValueError(f"Non-finite values in motion-prior term {name!r}")
            _append_with_backfill(self._histories[name], value)
        motion_prior = np.concatenate(
            [
                np.stack(self._histories[name], axis=0).reshape(-1)
                for name in (
                    "gravity",
                    "ang_vel",
                    "joint_pos",
                    "joint_vel",
                    "action",
                )
            ]
        ).astype(np.float32)
        if motion_prior.shape != (465,):
            raise RuntimeError(
                f"Built motion prior {motion_prior.shape}, expected (465,)"
            )
        return motion_prior

    def commit_applied_action(self, action: np.ndarray) -> None:
        value = np.asarray(action, dtype=np.float32).reshape(29)
        if not np.isfinite(value).all():
            raise ValueError("Applied action contains non-finite values")
        self.last_applied_action = value.copy()


class DualEncoderAdapter:
    EXPECTED_INPUTS = {
        "encoder_motion": [1, 523],
        "encoder_velocity": [1, 468],
        "encoder_mask": [1, 2],
        "motion_prior": [1, 465],
        "encoder_depth": [1, 1, 36, 64],
    }
    EXPECTED_OUTPUT = ("actions", [1, 29])

    def __init__(
        self,
        *,
        onnx_path: str | Path,
        controller_joint_names: Sequence[str],
        reference_fps: float = 50.0,
        input_timeout_s: float = 0.25,
        stick_deadband: float = 0.1,
        max_vx: float = 1.0,
        max_vy: float = 1.0,
        max_wz: float = 0.5,
        terrain_class: int = 0,
        providers: Sequence[str] | None = None,
        session: Any | None = None,
    ) -> None:
        self.controller_joint_names = tuple(
            str(name) for name in controller_joint_names
        )
        self._canonical_from_controller = _name_indices(
            self.controller_joint_names, CANONICAL_JOINT_NAMES
        )
        self._controller_from_canonical = _name_indices(
            CANONICAL_JOINT_NAMES, self.controller_joint_names
        )
        self.reference_fps = float(reference_fps)
        self.input_timeout_s = float(input_timeout_s)
        self.stick_deadband = float(stick_deadband)
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_wz = float(max_wz)
        self.terrain_class = int(terrain_class)
        if self.reference_fps <= 0.0:
            raise ValueError("reference_fps must be positive")
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be positive")
        if not 0.0 <= self.stick_deadband < 1.0:
            raise ValueError("stick_deadband must be in [0,1)")
        if min(self.max_vx, self.max_vy, self.max_wz) < 0.0:
            raise ValueError("Velocity command limits must be non-negative")

        if session is None:
            options = ort.SessionOptions()
            options.intra_op_num_threads = 4
            options.inter_op_num_threads = 1
            selected_providers = (
                list(providers) if providers is not None else ["CPUExecutionProvider"]
            )
            session = ort.InferenceSession(
                str(Path(onnx_path).expanduser().resolve()),
                sess_options=options,
                providers=selected_providers,
            )
        self.session = session
        self._validate_onnx_contract()
        self.observation = DualEncoderObservationBuilder()
        self.last_inputs: dict[str, np.ndarray] | None = None
        self._reference_mapping_key: tuple[str, ...] | None = None
        self._canonical_from_reference: np.ndarray | None = None

    def _validate_onnx_contract(self) -> None:
        inputs = {
            item.name: (list(item.shape), item.type)
            for item in self.session.get_inputs()
        }
        if set(inputs) != set(self.EXPECTED_INPUTS):
            raise ValueError(
                "Dual encoder ONNX inputs are "
                f"{sorted(inputs)}, expected {sorted(self.EXPECTED_INPUTS)}"
            )
        for name, expected_shape in self.EXPECTED_INPUTS.items():
            shape, dtype = inputs[name]
            if shape != expected_shape or dtype != "tensor(float)":
                raise ValueError(
                    f"Dual encoder ONNX input {name!r} is {(shape, dtype)}, "
                    f"expected {(expected_shape, 'tensor(float)')}"
                )
        outputs = self.session.get_outputs()
        expected_name, expected_shape = self.EXPECTED_OUTPUT
        if (
            len(outputs) != 1
            or outputs[0].name != expected_name
            or list(outputs[0].shape) != expected_shape
            or outputs[0].type != "tensor(float)"
        ):
            raise ValueError(
                "Dual encoder ONNX output must be "
                f"{self.EXPECTED_OUTPUT} tensor(float), got "
                f"{[(item.name, item.shape, item.type) for item in outputs]}"
            )

    def reset(self) -> None:
        self.observation.reset()
        self.last_inputs = None

    @staticmethod
    def _fresh(timestamp: float, timeout_s: float, now: float) -> bool:
        age = now - float(timestamp)
        return -0.1 <= age <= timeout_s

    def _pico_validity(
        self, snapshot: PicoSnapshot, *, now: float
    ) -> tuple[bool, bool]:
        pose_valid = bool(
            snapshot.active
            and snapshot.has_pose
            and self._fresh(snapshot.pose_timestamp, self.input_timeout_s, now)
        )
        control_valid = bool(
            snapshot.active
            and self._fresh(
                snapshot.control_timestamp, self.input_timeout_s, now
            )
        )
        return pose_valid, control_valid

    def _reference_joint_state(
        self, reference_policy
    ) -> tuple[np.ndarray | None, np.ndarray | None, str | None]:
        values = getattr(reference_policy, "ref_joint_pos", None)
        ref_len = int(getattr(reference_policy, "ref_len", 0))
        ref_idx = int(getattr(reference_policy, "ref_idx", 0))
        if values is None or ref_len <= 0:
            return None, None, "reference_empty"
        source = np.asarray(values, dtype=np.float32)
        if source.ndim != 2 or source.shape[0] != ref_len:
            return None, None, "reference_shape"
        if ref_idx < 0 or ref_idx >= ref_len:
            return None, None, "reference_index"
        if ref_len - 1 - ref_idx < 3:
            return None, None, "reference_future_horizon"

        reference_names = tuple(
            str(name) for name in getattr(reference_policy, "obs_joint_names", ())
        )
        if reference_names != self._reference_mapping_key:
            self._canonical_from_reference = _name_indices(
                reference_names, CANONICAL_JOINT_NAMES
            )
            self._reference_mapping_key = reference_names
        assert self._canonical_from_reference is not None
        canonical = source[:, self._canonical_from_reference]
        if not np.isfinite(canonical).all():
            return None, None, "reference_nonfinite"
        return (
            canonical[ref_idx].astype(np.float32, copy=True),
            reference_joint_velocity(canonical, ref_idx, self.reference_fps),
            None,
        )

    def _axis(self, snapshot: PicoSnapshot, name: str) -> float:
        value = float(np.clip(snapshot.sticks.get(name, 0.0), -1.0, 1.0))
        return 0.0 if abs(value) < self.stick_deadband else value

    def _velocity_command(self, snapshot: PicoSnapshot) -> np.ndarray:
        vx = self.max_vx * self._axis(snapshot, "ly")
        vy = self.max_vy * self._axis(snapshot, "lx")
        wz = self.max_wz * self._axis(snapshot, "rx")
        if self.terrain_class == 2:
            if vx <= 0.0:
                vx = 0.0
            else:
                vx = float(np.clip(vx, min(0.3, self.max_vx), self.max_vx))
            vy = 0.0
            wz = 0.0
        return np.asarray([vx, vy, wz], dtype=np.float32)

    @staticmethod
    def _projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
        quat = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm < 1.0e-6:
            raise ValueError("Controller quaternion is invalid")
        return R.from_quat(quat / norm, scalar_first=True).inv().apply(
            np.array([0.0, 0.0, -1.0], dtype=np.float64)
        ).astype(np.float32)

    def _observe_controller(self, controller) -> np.ndarray:
        q = np.asarray(controller.qj, dtype=np.float32)[
            self._canonical_from_controller
        ]
        dq = np.asarray(controller.dqj, dtype=np.float32)[
            self._canonical_from_controller
        ]
        return self.observation.observe(
            projected_gravity=self._projected_gravity(controller.quat),
            base_ang_vel=np.asarray(controller.gyro, dtype=np.float32),
            joint_pos=q,
            joint_vel=dq,
        )

    @staticmethod
    def _depth_input(depth: np.ndarray | None) -> np.ndarray:
        if depth is None:
            raise ValueError("velocity mode requires a depth frame")
        values = np.asarray(depth, dtype=np.float32)
        if values.shape not in ((1, 36, 64), (1, 1, 36, 64)):
            raise ValueError(
                "Depth must have shape (1,36,64) or (1,1,36,64), "
                f"got {values.shape}"
            )
        values = values.reshape(1, 1, 36, 64)
        if not np.isfinite(values).all():
            raise ValueError("Depth contains non-finite values")
        return values

    @staticmethod
    def _normalize_mode(input_mode: str) -> InputMode:
        mode = str(input_mode).strip().lower()
        if mode not in ("motion", "velocity"):
            raise ValueError(
                f"input_mode must be 'motion' or 'velocity', got {input_mode!r}"
            )
        return mode  # type: ignore[return-value]

    def step(
        self,
        *,
        controller,
        reference_policy,
        snapshot: PicoSnapshot,
        depth: np.ndarray | None,
        input_mode: str,
    ) -> DualEncoderStepResult:
        mode = self._normalize_mode(input_mode)
        try:
            motion_prior = self._observe_controller(controller)
        except (TypeError, ValueError, IndexError) as exc:
            return DualEncoderStepResult(None, False, str(exc), mode)

        now = time.monotonic()
        pose_valid, control_valid = self._pico_validity(snapshot, now=now)
        if not control_valid:
            return DualEncoderStepResult(None, False, "pico_stale", mode)

        if mode == "motion":
            if not pose_valid:
                return DualEncoderStepResult(None, False, "pico_stale", mode)
            ref_q, ref_dq, error = self._reference_joint_state(reference_policy)
            if error is not None:
                return DualEncoderStepResult(None, False, error, mode)
            assert ref_q is not None and ref_dq is not None
            velocity_command = np.zeros(3, dtype=np.float32)
            encoder_mask = np.array([1.0, 0.0], dtype=np.float32)
            depth_input = np.zeros((1, 1, 36, 64), dtype=np.float32)
        else:
            if depth is None:
                return DualEncoderStepResult(None, False, "depth_missing", mode)
            ref_q = DEFAULT_JOINT_POS.copy()
            ref_dq = np.zeros(29, dtype=np.float32)
            velocity_command = self._velocity_command(snapshot)
            encoder_mask = np.array([0.0, 1.0], dtype=np.float32)
            try:
                depth_input = self._depth_input(depth)
            except ValueError as exc:
                return DualEncoderStepResult(None, False, str(exc), mode)

        encoder_motion = np.concatenate((ref_q, ref_dq, motion_prior)).astype(
            np.float32
        )
        encoder_velocity = np.concatenate(
            (velocity_command, motion_prior)
        ).astype(np.float32)
        inputs = {
            "encoder_motion": np.ascontiguousarray(
                encoder_motion.reshape(1, 523)
            ),
            "encoder_velocity": np.ascontiguousarray(
                encoder_velocity.reshape(1, 468)
            ),
            "encoder_mask": np.ascontiguousarray(encoder_mask.reshape(1, 2)),
            "motion_prior": np.ascontiguousarray(motion_prior.reshape(1, 465)),
            "encoder_depth": np.ascontiguousarray(depth_input),
        }
        self.last_inputs = {name: value.copy() for name, value in inputs.items()}
        try:
            outputs = self.session.run(["actions"], inputs)
        except Exception as exc:
            return DualEncoderStepResult(
                None, False, f"onnx_inference_failed: {exc}", mode
            )
        if len(outputs) != 1:
            return DualEncoderStepResult(None, False, "onnx_output_count", mode)
        action = np.asarray(outputs[0], dtype=np.float32)
        if action.shape != (1, 29):
            return DualEncoderStepResult(
                None, False, f"onnx_output_shape: {action.shape}", mode
            )
        raw_action = action[0]
        if not np.isfinite(raw_action).all():
            return DualEncoderStepResult(None, False, "onnx_output_nonfinite", mode)

        q_des_canonical = DEFAULT_JOINT_POS + ACTION_SCALE * raw_action
        indexes = self._controller_from_canonical
        command = ControlCommand(
            q_des=q_des_canonical[indexes].astype(np.float32),
            kp=JOINT_KP[indexes].astype(np.float32),
            kd=JOINT_KD[indexes].astype(np.float32),
        )
        return DualEncoderStepResult(command, True, "", mode)

    def commit_applied_target(self, q_des_controller: np.ndarray) -> None:
        q_des = np.asarray(q_des_controller, dtype=np.float32)
        if q_des.shape != (len(self.controller_joint_names),):
            raise ValueError(
                f"Applied target is {q_des.shape}, expected "
                f"{(len(self.controller_joint_names),)}"
            )
        canonical = q_des[self._canonical_from_controller]
        raw_action = (canonical - DEFAULT_JOINT_POS) / ACTION_SCALE
        self.observation.commit_applied_action(raw_action)


class DualTeacherController(Controller):
    """Standalone deployment loop for the dual-teacher distilled policy.

    ``Controller`` and its existing tracking policy remain the owners of robot
    state, PICO transport, retarget buffering and reference joint ordering.
    Tracking also remains the balancing fallback until the dual policy is ready
    and whenever a live dual input becomes stale.
    """

    def __init__(
        self,
        args,
        ctrl_cfg,
        task_config: dict,
        task_config_path: Path,
    ) -> None:
        self.pico_store = PicoFrameStore()
        args.pico_store = self.pico_store
        args.force_vr_motion_source = True
        super().__init__(args, ctrl_cfg)

        self.task_config = task_config
        configured_mode = task_config.get("input_mode", "motion")
        self.input_mode: InputMode = DualEncoderAdapter._normalize_mode(
            configured_mode if args.input_mode is None else args.input_mode
        )
        self.target = str(task_config["target"])
        self.reference_policy = self.policies["tracking"]
        velocity_cfg = task_config.get("velocity_command", {})
        if not isinstance(velocity_cfg, dict):
            raise ValueError("velocity_command must be a mapping")
        terrain_class = (
            int(velocity_cfg.get("terrain_class", 2))
            if args.terrain_class is None
            else int(args.terrain_class)
        )
        onnx_path = _resolve(task_config_path, task_config["dual_onnx"])
        input_timeout_s = float(task_config.get("input_timeout_s", 0.25))
        handoff_cfg = task_config.get("handoff", {})
        if not isinstance(handoff_cfg, dict):
            raise ValueError("handoff must be a mapping")
        blend_duration_s = float(handoff_cfg.get("blend_duration_s", 1.0))
        ready_hold_s = float(handoff_cfg.get("ready_hold_s", 0.1))
        if blend_duration_s < 0.0:
            raise ValueError("handoff.blend_duration_s must be non-negative")
        if ready_hold_s < 0.0:
            raise ValueError("handoff.ready_hold_s must be non-negative")
        self.blend_steps = max(
            1, int(round(blend_duration_s / self.control_dt))
        )
        self.ready_steps = max(1, int(round(ready_hold_s / self.control_dt)))
        self._dual_ready_steps = 0
        self._control_source: ControlSource = "tracking"
        self._blend_step = self.blend_steps
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        self.adapter = DualEncoderAdapter(
            onnx_path=onnx_path,
            controller_joint_names=self.policy_joint_names,
            reference_fps=float(task_config.get("reference_fps", 50.0)),
            input_timeout_s=input_timeout_s,
            stick_deadband=float(velocity_cfg.get("deadband", 0.1)),
            max_vx=float(velocity_cfg.get("max_vx", 1.0)),
            max_vy=float(velocity_cfg.get("max_vy", 1.0)),
            max_wz=float(velocity_cfg.get("max_wz", 0.5)),
            terrain_class=terrain_class,
        )

        self.depth_sub = None
        self.depth_processor = None
        self.raw_depth_shape = None
        self.depth_timeout_s = input_timeout_s
        self.max_invalid_fraction = 0.6
        self.depth_connect = None
        if self.input_mode == "velocity":
            camera_cfg = task_config.get("camera_process")
            if not isinstance(camera_cfg, dict):
                raise ValueError(
                    "velocity input_mode requires camera_process mapping"
                )
            self.depth_timeout_s = float(
                camera_cfg.get("timeout_s", input_timeout_s)
            )
            self.max_invalid_fraction = float(
                camera_cfg.get("max_invalid_fraction", 0.6)
            )
            if not 0.0 <= self.max_invalid_fraction <= 1.0:
                raise ValueError("max_invalid_fraction must be in [0,1]")
            self.depth_connect = str(
                camera_cfg["depth_connect"]
                if args.depth_connect is None
                else args.depth_connect
            )
            self.depth_sub = ArraySubscriber(self.depth_connect, topic="depth")
            if self.target == "real":
                preprocess_cfg = camera_cfg.get("preprocess")
                hardware_cfg = camera_cfg.get("hardware")
                if not isinstance(preprocess_cfg, dict):
                    raise ValueError(
                        "real velocity mode requires camera_process.preprocess"
                    )
                if not isinstance(hardware_cfg, dict):
                    raise ValueError(
                        "real velocity mode requires camera_process.hardware"
                    )
                self.depth_processor = RealDepthProcessor.from_config(
                    preprocess_cfg
                )
                self.raw_depth_shape = (
                    int(hardware_cfg.get("height", 360)),
                    int(hardware_cfg.get("width", 640)),
                )
                if min(self.raw_depth_shape) <= 0:
                    raise ValueError("raw depth dimensions must be positive")

        depth_summary = (
            "masked (camera not required)"
            if self.depth_sub is None
            else self.depth_connect
        )
        print(
            "[DualTeacher] "
            f"target={self.target}, input_mode={self.input_mode}, "
            f"task_config={task_config_path}, onnx={onnx_path}, "
            f"reference_source=existing PICO tracking chain, "
            f"depth={depth_summary}, fallback=tracking, "
            f"blend_steps={self.blend_steps}, ready_steps={self.ready_steps}"
        )
        self._last_wait_reason = None
        self._last_wait_report = 0.0

    def _read_depth(self) -> np.ndarray | None:
        if self.depth_sub is None:
            return None
        packet = self.depth_sub.read_latest()
        if packet is None:
            return None
        age = time.monotonic() - packet.recv_time
        if age < -0.1 or age > self.depth_timeout_s:
            return None

        if self.target == "real":
            if packet.metadata.get("protocol") != "d435i-raw-z16-v1":
                raise ValueError(
                    "Real depth stream must use protocol d435i-raw-z16-v1"
                )
            raw = np.asarray(packet.values)
            if raw.dtype != np.uint16 or raw.shape != self.raw_depth_shape:
                raise ValueError(
                    f"Raw depth {raw.dtype} {raw.shape} != "
                    f"uint16 {self.raw_depth_shape}"
                )
            assert self.depth_processor is not None
            depth, stats = self.depth_processor.process_raw(
                raw, depth_scale=float(packet.metadata["depth_scale"])
            )
            invalid_fraction = float(stats["invalid_fraction"])
        else:
            values = np.asarray(packet.values)
            if values.shape != (1, 36, 64):
                raise ValueError(
                    f"Sim depth {values.shape} != (1, 36, 64)"
                )
            depth = values.astype(np.float32, copy=False)
            invalid_value = float(
                packet.metadata.get("invalid_value", -1.0)
            )
            invalid_fraction = float(
                packet.metadata.get(
                    "invalid_fraction", np.mean(depth == invalid_value)
                )
            )
        if invalid_fraction > self.max_invalid_fraction:
            return None
        return depth

    def _report_wait(self, reason: str) -> None:
        now = time.monotonic()
        if reason != self._last_wait_reason or now - self._last_wait_report >= 1.0:
            print(f"[DualTeacher] tracking fallback: {reason}")
            self._last_wait_reason = reason
            self._last_wait_report = now

    def _tracking_command(self, reference) -> ControlCommand:
        action_delta = np.asarray(
            reference.compute_action(), dtype=np.float32
        )
        expected_shape = (len(self.policy_joint_names),)
        if action_delta.shape != expected_shape:
            raise ValueError(
                f"Tracking fallback action is {action_delta.shape}, "
                f"expected {expected_shape}"
            )
        if not np.isfinite(action_delta).all():
            raise ValueError("Tracking fallback action contains non-finite values")
        return ControlCommand(
            q_des=(self.default_qpos + action_delta).astype(np.float32),
            kp=self.kps.astype(np.float32, copy=True),
            kd=self.kds.astype(np.float32, copy=True),
        )

    def _set_control_source(self, source: ControlSource) -> None:
        if source == self._control_source:
            return
        self._control_source = source
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        self._blend_step = 0
        print(f"[DualTeacher] switched -> {source}")

    def _update_control_source(self, *, dual_ready: bool) -> None:
        if dual_ready:
            self._dual_ready_steps = min(
                self._dual_ready_steps + 1, self.ready_steps
            )
            if (
                self._control_source == "tracking"
                and self._dual_ready_steps >= self.ready_steps
            ):
                self._set_control_source("dual")
            return

        self._dual_ready_steps = 0
        if self._control_source == "dual":
            self._set_control_source("tracking")

    def _apply_command(self, command: ControlCommand) -> ControlCommand:
        if self._blend_step < self.blend_steps:
            self._blend_step += 1
            alpha = self._blend_step / self.blend_steps
            q_des = (
                self._blend_start.q_des * (1.0 - alpha)
                + command.q_des * alpha
            )
            kp = self._blend_start.kp * (1.0 - alpha) + command.kp * alpha
            kd = self._blend_start.kd * (1.0 - alpha) + command.kd * alpha
        else:
            q_des, kp, kd = command.q_des, command.kp, command.kd
        self.cmd_q[:] = q_des
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = kp
        self.cmd_kd[:] = kd
        self.cmd_enable = 1
        return ControlCommand(
            q_des=np.asarray(q_des, dtype=np.float32).copy(),
            kp=np.asarray(kp, dtype=np.float32).copy(),
            kd=np.asarray(kd, dtype=np.float32).copy(),
        )

    def _commit_applied_command(self, command: ControlCommand, reference) -> None:
        self.adapter.commit_applied_target(command.q_des)

        default = (
            self.default_qpos
            if reference.controller_default_qpos is None
            else reference.controller_default_qpos
        )
        delta_controller = np.asarray(command.q_des) - np.asarray(default)
        scaled_policy = reference.mapper_action.map_state_to_from(
            delta_controller
        ).astype(np.float32)
        scale = np.asarray(reference.action_scale, dtype=np.float32)
        raw = np.divide(
            scaled_policy,
            scale,
            out=np.zeros_like(scaled_policy),
            where=np.abs(scale) > 1.0e-8,
        )
        raw = np.clip(raw, -reference.action_clip, reference.action_clip)
        reference.last_action[:] = raw
        reference.applied_action[:] = raw * scale

    def run_dual_teacher(self) -> None:
        reference = self.reference_policy
        # Match the other task handoff: start from the command that is actually
        # holding the robot, run tracking as the balancing fallback, and blend
        # only after the dual inputs have remained ready for the configured
        # number of policy steps.
        self._control_source = "tracking"
        self._dual_ready_steps = 0
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        self._blend_step = 0
        print(
            "[DualTeacher] running; robot/sim stop exits, PICO right A "
            "starts or realigns the reference stream; tracking remains the "
            "fallback until dual inputs are ready"
        )
        while True:
            if not self.process_state(wait_next=True, timeout_s=1.0):
                if self.target == "real":
                    raise TimeoutError("No bridge state for 1s")
                print("[DualTeacher] no bridge state for 1s")
                continue
            if self.btn_rise["stop"]:
                break

            reference.source.poll_control()
            reference.update_obs()
            tracking_command = self._tracking_command(reference)
            snapshot = self.pico_store.snapshot()
            result = self.adapter.step(
                controller=self,
                reference_policy=reference,
                snapshot=snapshot,
                depth=self._read_depth(),
                input_mode=self.input_mode,
            )
            if result.ready:
                assert result.command is not None
                if self._last_wait_reason is not None:
                    print("[DualTeacher] inputs ready")
                    self._last_wait_reason = None
            else:
                self._report_wait(result.reason)

            self._update_control_source(dual_ready=result.ready)
            target_command = (
                result.command
                if self._control_source == "dual" and result.command is not None
                else tracking_command
            )
            applied_command = self._apply_command(target_command)
            self._commit_applied_command(applied_command, reference)
            self.send_cmd()
            reference.post_step()
            self.policy_step += 1

    def close_dual_teacher(self) -> None:
        try:
            self.reference_policy.deactivate()
        finally:
            if self.depth_sub is not None:
                self.depth_sub.close()
            self.transport.close()


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Standalone adapter for the dual-teacher distilled ONNX"
    )
    parser.add_argument("--robot", choices=("g1",), default="g1")
    parser.add_argument("--target", choices=("sim", "real"), default="sim")
    parser.add_argument("--task-config", type=Path, default=None)
    parser.add_argument(
        "--input-mode",
        choices=("motion", "velocity"),
        default=None,
        help="Optional override of task-config input_mode",
    )
    parser.add_argument(
        "--tracking-config",
        default=None,
        help="Optional override of task-config tracking_config",
    )
    parser.add_argument("--controller-config", type=Path, default=None)
    parser.add_argument(
        "--terrain-class", type=int, choices=(0, 1, 2), default=None
    )
    parser.add_argument("--depth-connect", default=None)
    return parser


def main(argv=None) -> None:
    args = _argument_parser().parse_args(argv)
    args.no_record = True
    args.record_dir = None
    if args.task_config is None:
        config_name = (
            "dual-teacher-real.yaml"
            if args.target == "real"
            else "dual-teacher.yaml"
        )
        args.task_config = SIM2REAL_ROOT / "config/g1" / config_name
    task_config_path = args.task_config.expanduser().resolve()
    task_config = _load_task_config(task_config_path)
    configured_target = str(task_config.get("target", args.target))
    if configured_target != args.target:
        raise ValueError(
            f"--target {args.target} does not match task config target "
            f"{configured_target!r}: {task_config_path}"
        )
    args.tracking_config = str(
        task_config.get("tracking_config", "tracking.yaml")
        if args.tracking_config is None
        else args.tracking_config
    )
    controller = None
    try:
        config_path = (
            args.controller_config
            if args.controller_config is not None
            else controller_config_path(args.robot)
        )
        controller = DualTeacherController(
            args,
            get_config(config_path),
            task_config,
            task_config_path,
        )
        controller.zero_torque_state()
        controller.move_to_default_qpos()
        controller.default_qpos_state()
        controller.run_dual_teacher()
    except KeyboardInterrupt:
        print("[DualTeacher] interrupted")
    except Exception as exc:
        print(f"[DualTeacher] error: {exc}")
        traceback.print_exc()
    finally:
        if controller is not None:
            controller.close_dual_teacher()


__all__ = [
    "ACTION_SCALE",
    "CANONICAL_JOINT_NAMES",
    "ControlCommand",
    "DEFAULT_JOINT_POS",
    "DualEncoderAdapter",
    "DualEncoderObservationBuilder",
    "DualEncoderStepResult",
    "DualTeacherController",
    "JOINT_KD",
    "JOINT_KP",
    "reference_joint_velocity",
]


if __name__ == "__main__":
    main()
