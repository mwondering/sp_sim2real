import json
import os
import time
from abc import ABC
from pathlib import Path
from typing import TYPE_CHECKING, Dict, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from runtime.math_utils import _linspace_rows, _slerp, _yaw_component_wxyz
from runtime.shared_pico import PicoFrameStore
from common.udp_latest import LatestPacket, UDPLatestReceiver
from common.utils import DictToClass

try:
    import zmq
except Exception:
    zmq = None

if TYPE_CHECKING:
    from runtime.policy import TrackingPolicyRaw


# IsaacLab stores G1 joints in this articulation order.  SP_Tracking's Sonic
# datasets use the same order but historically did not embed joint_names in
# each NPZ, so the order must be part of the deployment contract.
ISAACLAB_G1_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)


def reference_endpoint_from_env(
    *,
    default: str,
    explicit_env: str,
    port_env: str,
) -> str:
    """Resolve one pico-branch endpoint without mutating checked-in YAML."""
    explicit = os.environ.get(explicit_env, "").strip()
    if explicit:
        return explicit
    reference_host = os.environ.get("G1_REF_HOST", "").strip()
    if not reference_host:
        return default
    try:
        port = int(os.environ[port_env])
    except KeyError:
        port = int(default.rsplit(":", 1)[1])
    if port < 1 or port > 65535:
        raise ValueError(f"{port_env} must be in [1, 65535]")
    return f"tcp://{reference_host}:{port}"


def _decode_joint_names(values) -> list[str]:
    names = []
    for name in np.asarray(values).reshape(-1).tolist():
        if isinstance(name, (bytes, np.bytes_)):
            names.append(name.decode("utf-8"))
        else:
            names.append(str(name))
    return names


def _normalize_quaternions_wxyz(values: np.ndarray, *, motion_name: str) -> np.ndarray:
    quaternions = np.asarray(values, dtype=np.float32)
    if quaternions.ndim != 2 or quaternions.shape[1] != 4:
        raise ValueError(
            f"Motion '{motion_name}' root quaternion must have shape [T, 4], "
            f"got {quaternions.shape}."
        )
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms < 1.0e-6):
        raise ValueError(f"Motion '{motion_name}' contains a zero-length root quaternion.")
    return (quaternions / norms).astype(np.float32)


def _validate_motion_arrays(
    *,
    motion_name: str,
    joint_pos: np.ndarray,
    root_pos: np.ndarray,
    root_quat: np.ndarray,
) -> None:
    if joint_pos.ndim != 2:
        raise ValueError(f"Motion '{motion_name}' joint_pos must be [T, J], got {joint_pos.shape}.")
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"Motion '{motion_name}' root_pos must be [T, 3], got {root_pos.shape}.")
    frame_count = joint_pos.shape[0]
    if frame_count == 0:
        raise ValueError(f"Motion '{motion_name}' has no frames after applying start/end.")
    if root_pos.shape[0] != frame_count or root_quat.shape[0] != frame_count:
        raise ValueError(
            f"Motion '{motion_name}' frame mismatch: joint={joint_pos.shape}, "
            f"root_pos={root_pos.shape}, root_quat={root_quat.shape}."
        )
    for field_name, values in (
        ("joint_pos", joint_pos),
        ("root_pos", root_pos),
        ("root_quat", root_quat),
    ):
        if not np.isfinite(values).all():
            raise ValueError(f"Motion '{motion_name}' contains non-finite {field_name} values.")


def _motion_frame_slice(start: int, end: int) -> slice:
    # Repository configs have always documented end=-1 as "through the end".
    # Convert it to Python's open-ended slice rather than silently dropping the
    # final frame.
    return slice(int(start), None if int(end) == -1 else int(end))


def motion_name_from_path(path: Path, root: Path) -> str:
    """Return the stable selector name for one motion below ``root``."""
    path = path.expanduser().resolve()
    root = root.expanduser().resolve()
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"Motion is outside motion_root={root}: {path}") from exc
    return relative.with_suffix("").as_posix()


def discover_motion_files(root: Path) -> Dict[str, Path]:
    """Discover local NPZ motions without loading their frame arrays."""
    root = root.expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"motion_root is not a directory: {root}")

    motions: Dict[str, Path] = {}
    for path in sorted(root.rglob("*.npz")):
        if not path.is_file():
            continue
        resolved = path.resolve()
        name = motion_name_from_path(resolved, root)
        if name == "default":
            raise ValueError(
                f"Local motion name 'default' is reserved for the configured default clip: {resolved}"
            )
        if name in motions:
            raise ValueError(f"Duplicate local motion name {name!r}: {motions[name]} and {resolved}")
        motions[name] = resolved
    return motions


def _load_npz_motion(
    data: np.lib.npyio.NpzFile,
    *,
    motion_name: str,
    frame_slice: slice,
    motion_type: str,
    dataset_joint_names: Sequence[str],
    root_body_index: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str], str]:
    fields = set(data.files)
    legacy_fields = {"dof_pos", "root_pos", "root_rot"}
    isaaclab_fields = {"joint_pos", "body_pos_w", "body_quat_w"}

    if legacy_fields.issubset(fields):
        joint_pos = np.asarray(data["dof_pos"][frame_slice], dtype=np.float32)
        root_pos = np.asarray(data["root_pos"][frame_slice], dtype=np.float32)
        root_rot_xyzw = np.asarray(data["root_rot"][frame_slice], dtype=np.float32)
        if root_rot_xyzw.ndim != 2 or root_rot_xyzw.shape[1] != 4:
            raise ValueError(
                f"Motion '{motion_name}' root_rot must have shape [T, 4], "
                f"got {root_rot_xyzw.shape}."
            )
        root_quat = np.concatenate(
            [root_rot_xyzw[:, 3:4], root_rot_xyzw[:, :3]], axis=-1
        )
        if "joint_names" not in fields:
            raise ValueError(
                f"Motion '{motion_name}' uses the legacy NPZ schema but has no joint_names."
            )
        source_joint_names = _decode_joint_names(data["joint_names"])
        schema = "legacy"
    elif isaaclab_fields.issubset(fields):
        joint_pos = np.asarray(data["joint_pos"][frame_slice], dtype=np.float32)
        body_pos_w = np.asarray(data["body_pos_w"][frame_slice], dtype=np.float32)
        body_quat_w = np.asarray(data["body_quat_w"][frame_slice], dtype=np.float32)
        if body_pos_w.ndim != 3 or body_pos_w.shape[2] != 3:
            raise ValueError(
                f"Motion '{motion_name}' body_pos_w must have shape [T, B, 3], "
                f"got {body_pos_w.shape}."
            )
        if body_quat_w.ndim != 3 or body_quat_w.shape[2] != 4:
            raise ValueError(
                f"Motion '{motion_name}' body_quat_w must have shape [T, B, 4], "
                f"got {body_quat_w.shape}."
            )
        if body_pos_w.shape[:2] != body_quat_w.shape[:2]:
            raise ValueError(
                f"Motion '{motion_name}' body pose/quaternion dimensions do not match: "
                f"body_pos_w={body_pos_w.shape}, body_quat_w={body_quat_w.shape}."
            )
        if root_body_index < 0 or root_body_index >= body_pos_w.shape[1]:
            raise ValueError(
                f"Motion '{motion_name}' root_body_index={root_body_index} is outside "
                f"the body dimension {body_pos_w.shape[1]}."
            )
        root_pos = body_pos_w[:, root_body_index]
        # IsaacLab/SP_Tracking body_quat_w is already scalar-first (wxyz).
        root_quat = body_quat_w[:, root_body_index]
        if "joint_names" in fields:
            source_joint_names = _decode_joint_names(data["joint_names"])
            joint_order = "embedded"
        elif motion_type == "mujoco":
            source_joint_names = [str(name) for name in dataset_joint_names]
            joint_order = "mujoco"
        else:
            # auto intentionally selects IsaacLab for this schema: these field
            # names are the standard IsaacLab/SP_Tracking export contract.
            source_joint_names = list(ISAACLAB_G1_JOINT_NAMES)
            joint_order = "isaaclab"
        schema = f"isaaclab/{joint_order}"
    else:
        raise ValueError(
            f"Motion '{motion_name}' has unsupported NPZ fields. Expected either "
            f"{sorted(legacy_fields)} or {sorted(isaaclab_fields)}, got {sorted(fields)}."
        )

    root_quat = _normalize_quaternions_wxyz(root_quat, motion_name=motion_name)
    _validate_motion_arrays(
        motion_name=motion_name,
        joint_pos=joint_pos,
        root_pos=root_pos,
        root_quat=root_quat,
    )
    return joint_pos, root_pos, root_quat, source_joint_names, schema


def remap_joint_array_by_names(
    data: np.ndarray,
    source_joint_names,
    target_joint_names,
) -> np.ndarray:
    data = np.asarray(data, dtype=np.float32)
    if data.ndim != 2:
        raise ValueError(f"Expected 2D joint array [T, J], got shape={data.shape}")
    if data.shape[1] != len(source_joint_names):
        raise ValueError(
            f"Joint dim mismatch: data has {data.shape[1]} dims, "
            f"but source_joint_names has {len(source_joint_names)} names."
        )

    source_joint_names = [str(name) for name in source_joint_names]
    target_joint_names = [str(name) for name in target_joint_names]
    if len(set(source_joint_names)) != len(source_joint_names):
        raise ValueError("source_joint_names contains duplicates")
    name_to_idx = {name: i for i, name in enumerate(source_joint_names)}
    missing = [name for name in target_joint_names if name not in name_to_idx]
    if missing:
        raise ValueError(f"Motion is missing target joints: {missing}")
    remap = np.zeros((data.shape[0], len(target_joint_names)), dtype=np.float32)
    for i, name in enumerate(target_joint_names):
        j = name_to_idx.get(name, None)
        if j is not None:
            remap[:, i] = data[:, j]
    return remap


class MotionSourceBase(ABC):
    def __init__(self, policy: "TrackingPolicyRaw", policy_cfg: DictToClass):
        self.policy = policy
        self.config = policy_cfg
        self.motions: Dict[str, Dict[str, np.ndarray]] = self._load_motions()

    def _load_motions(self) -> Dict[str, Dict[str, np.ndarray]]:
        motions: Dict[str, Dict[str, np.ndarray]] = {}

        for m in getattr(self.config, "motions", []):
            mc = DictToClass(m)
            motion_name = mc.name
            mp = Path(mc.path)
            cfg_dir = Path(getattr(self.config, "_config_dir"))
            path = mp if mp.is_absolute() else (cfg_dir / mp)
            motions[motion_name] = self._load_motion_file(
                motion_name,
                path,
                start=int(getattr(mc, "start", 0)),
                end=int(getattr(mc, "end", -1)),
                motion_type=str(
                    getattr(mc, "motion_type", getattr(self.config, "motion_type", "auto"))
                ),
                root_body_index=int(
                    getattr(mc, "root_body_index", getattr(self.config, "root_body_index", 0))
                ),
            )

        for m in getattr(self.config, "motion_clips", []):
            mc = DictToClass(m)
            motion_name = mc.name
            joint_pos_1 = np.asarray(mc.joint_pos, dtype=np.float32).reshape(1, -1)
            if joint_pos_1.shape[1] != len(self.policy.dataset_joint_names):
                raise ValueError(
                    f"[{self.__class__.__name__}] Motion clip '{motion_name}' dim={joint_pos_1.shape[1]} "
                    f"does not match dataset_joint_names size={len(self.policy.dataset_joint_names)}."
                )
            source_joint_names = self.policy.dataset_joint_names
            joint_pos_1 = remap_joint_array_by_names(joint_pos_1, source_joint_names, self.policy.obs_joint_names)
            root_quat_1 = np.asarray(mc.root_quat, dtype=np.float32).reshape(1, 4)
            root_pos_1 = np.asarray(mc.root_pos, dtype=np.float32).reshape(1, 3)

            motions[motion_name] = {
                "joint_pos": joint_pos_1,
                "root_quat": root_quat_1,
                "root_pos": root_pos_1,
            }

        if "default" not in motions:
            raise ValueError(f"[{self.__class__.__name__}] motions must include a 'default' clip (length==1).")

        return motions

    def _load_motion_file(
        self,
        motion_name: str,
        path: Path,
        *,
        start: int = 0,
        end: int = -1,
        motion_type: Optional[str] = None,
        root_body_index: Optional[int] = None,
    ) -> Dict[str, np.ndarray]:
        """Load and remap one motion on demand for the current policy."""
        path = path.expanduser().resolve()
        resolved_type = str(
            motion_type
            if motion_type is not None
            else getattr(self.config, "motion_type", "auto")
        ).strip().lower()
        if resolved_type not in ("auto", "isaaclab", "mujoco"):
            raise ValueError(
                f"Motion '{motion_name}' motion_type must be auto, isaaclab, or mujoco; "
                f"got {resolved_type!r}."
            )
        resolved_root_body_index = int(
            getattr(self.config, "root_body_index", 0)
            if root_body_index is None
            else root_body_index
        )

        with np.load(path, allow_pickle=True) as data:
            if not isinstance(data, np.lib.npyio.NpzFile):
                raise ValueError(f"[{self.__class__.__name__}] Only .npz is supported: {path}")
            joint_pos, root_pos, root_quat, source_joint_names, schema = _load_npz_motion(
                data,
                motion_name=motion_name,
                frame_slice=_motion_frame_slice(start, end),
                motion_type=resolved_type,
                dataset_joint_names=self.policy.dataset_joint_names,
                root_body_index=resolved_root_body_index,
            )
        joint_pos = remap_joint_array_by_names(
            joint_pos,
            source_joint_names,
            self.policy.obs_joint_names,
        )
        print(
            f"[{self.__class__.__name__}] Loaded motion '{motion_name}' "
            f"schema={schema}, frames={joint_pos.shape[0]}, path={path}"
        )
        return {
            "joint_pos": joint_pos,
            "root_quat": root_quat,
            "root_pos": root_pos,
        }

    @staticmethod
    def _empty_frames(n_joints: int) -> Dict[str, np.ndarray]:
        return {
            "joint_pos": np.zeros((0, n_joints), dtype=np.float32),
            "root_quat": np.zeros((0, 4), dtype=np.float32),
            "root_pos": np.zeros((0, 3), dtype=np.float32),
        }

    def _align_motion_to_anchor(
        self,
        motion: Dict[str, np.ndarray],
        anchor: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
        p0 = motion["root_pos"][0]
        q0_yaw = _yaw_component_wxyz(motion["root_quat"][0])
        pa = anchor["root_pos"]
        qa_yaw = _yaw_component_wxyz(anchor["root_quat"])

        r0 = R.from_quat(q0_yaw, scalar_first=True)
        ra = R.from_quat(qa_yaw, scalar_first=True)
        r_delta = ra * r0.inv()

        root_pos_aligned = r_delta.apply(motion["root_pos"] - p0) + pa
        root_pos_aligned[:, 2] = motion["root_pos"][:, 2]

        root_quat_all = R.from_quat(motion["root_quat"], scalar_first=True)
        root_quat_aligned = (r_delta * root_quat_all).as_quat(scalar_first=True)

        return {
            "joint_pos": motion["joint_pos"].astype(np.float32, copy=True),
            "root_quat": root_quat_aligned.astype(np.float32),
            "root_pos": root_pos_aligned.astype(np.float32),
        }

    def _build_transition_prefix(
        self,
        anchor: Dict[str, np.ndarray],
        tgt_first: Dict[str, np.ndarray],
    ) -> Dict[str, np.ndarray]:
        t_steps = int(self.policy.transition_steps)
        if t_steps <= 0:
            return self._empty_frames(self.policy.n_joints)

        joints_tr = _linspace_rows(anchor["joint_pos"], tgt_first["joint_pos"], t_steps)
        root_pos_tr = _linspace_rows(anchor["root_pos"], tgt_first["root_pos"], t_steps)
        root_quat_tr = _slerp(anchor["root_quat"], tgt_first["root_quat"], t_steps)

        return {
            "joint_pos": joints_tr,
            "root_quat": root_quat_tr,
            "root_pos": root_pos_tr,
        }

    def append_motion_from_tail(self, name: str) -> bool:
        if name not in self.motions:
            print(f"[{self.__class__.__name__}] Unknown motion '{name}'")
            return False

        anchor = self.policy.read_ref_tail_state()
        aligned_motion = self._align_motion_to_anchor(self.motions[name], anchor)

        tgt_first = {
            "joint_pos": aligned_motion["joint_pos"][0],
            "root_quat": aligned_motion["root_quat"][0],
            "root_pos": aligned_motion["root_pos"][0],
        }
        trans_motion = self._build_transition_prefix(anchor, tgt_first)

        segment = {
            "joint_pos": np.concatenate([trans_motion["joint_pos"], aligned_motion["joint_pos"]], axis=0),
            "root_quat": np.concatenate([trans_motion["root_quat"], aligned_motion["root_quat"]], axis=0),
            "root_pos": np.concatenate([trans_motion["root_pos"], aligned_motion["root_pos"]], axis=0),
        }
        self.policy.append_ref_frames(segment)

        self.policy.current_name = name
        self.policy.current_done = (self.policy.ref_idx >= self.policy.ref_len - 1)

        print(
            f"[{self.__class__.__name__}] Append motion '{name}' | appended={segment['joint_pos'].shape[0]}, "
            f"ref_len={self.policy.ref_len}, transition={self.policy.transition_steps}"
        )
        return True

    def on_fade_in(self):
        self.append_motion_from_tail("default")

    def on_fade_out(self):
        self.append_motion_from_tail("default")

    def deactivate(self):
        return

    def post_step(self):
        return


class UDPMotionSource(MotionSourceBase):
    def __init__(self, policy: "TrackingPolicyRaw", policy_cfg: DictToClass):
        udp_cfg = getattr(policy_cfg, "motion_source")["udp"]
        self.udp_enable = bool(udp_cfg["enable"])
        self.udp_host = str(udp_cfg["host"])
        self.udp_port = int(udp_cfg["port"])
        self.motion_root: Optional[Path] = None
        self.motion_files: Dict[str, Path] = {}
        configured_motion_root = str(udp_cfg.get("motion_root", "")).strip()
        if configured_motion_root:
            root = Path(configured_motion_root).expanduser()
            if not root.is_absolute():
                root = Path(getattr(policy_cfg, "_config_dir")) / root
            self.motion_root = root.resolve()
            self.motion_files = discover_motion_files(self.motion_root)
        self._udp_receiver: Optional[UDPLatestReceiver] = None
        self._latest_motion_packet: Optional[LatestPacket] = None
        self._latest_motion_seq: int = -1
        self._pending_local_motion: Optional[str] = None

        super().__init__(policy, policy_cfg)

        if self.motion_root is not None:
            print(
                f"[UDPMotionSource] Discovered {len(self.motion_files)} local motions "
                f"under {self.motion_root}; files are loaded only when selected"
            )

        if self.udp_enable:
            try:
                self._udp_receiver = UDPLatestReceiver(
                    self.udp_host,
                    self.udp_port,
                )
                self._udp_receiver.start()
            except Exception as e:
                self._udp_receiver = None
                print(f"[UDPMotionSource] Failed to start UDP server: {e}")

    def request_motion(self, name: str) -> bool:
        is_lazy_motion = name in self.motion_files and name not in self.motions
        if name not in self.motions and not is_lazy_motion:
            print(f"[UDPMotionSource] Unknown motion '{name}'")
            return False

        if not (
            (self.policy.current_name == "default" or name == "default")
            and self.policy.current_done
        ):
            print(
                f"[UDPMotionSource] Reject '{name}': "
                f"current='{self.policy.current_name}', done={self.policy.current_done}"
            )
            return False

        if is_lazy_motion:
            try:
                self.motions[name] = self._load_motion_file(name, self.motion_files[name])
            except (OSError, ValueError) as exc:
                print(f"[UDPMotionSource] Failed to load local motion '{name}': {exc}")
                return False

        appended = self.append_motion_from_tail(name)
        if is_lazy_motion:
            # append_motion_from_tail copies the aligned frames into the policy
            # buffer, so keeping the source arrays would only double onboard RAM.
            self.motions.pop(name, None)
        return appended

    def _queue_local_motion(self, name: str) -> bool:
        if name not in self.motions and name not in self.motion_files:
            print(f"[UDPMotionSource] Unknown onboard motion '{name}'")
            return False
        self._pending_local_motion = name
        print(f"[UDPMotionSource] Queued onboard motion '{name}'")
        return True

    def _advance_local_motion_queue(self) -> None:
        if self.motion_root is None or not self.policy.current_done:
            return

        if self.policy.current_name != "default":
            if self.append_motion_from_tail("default"):
                print("[UDPMotionSource] Onboard motion finished; returning to default")
                if self._pending_local_motion == "default":
                    self._pending_local_motion = None
            return

        if self._pending_local_motion is None:
            return
        name = self._pending_local_motion
        self._pending_local_motion = None
        self.request_motion(name)

    def post_step(self):
        if self._udp_receiver is not None:
            packet = self._udp_receiver.read_latest_data(with_meta=True)
            if packet is not None and packet.seq != self._latest_motion_seq:
                self._latest_motion_seq = packet.seq
                self._latest_motion_packet = packet
                payload = packet.data
                if isinstance(payload, dict):
                    cmd = str(payload.get("motion", "")).strip()
                else:
                    cmd = str(payload).strip()
                if cmd:
                    normalized = "default" if cmd == "default" else cmd
                    if self.motion_root is not None:
                        self._queue_local_motion(normalized)
                    else:
                        self.request_motion(normalized)

        self._advance_local_motion_queue()

    def deactivate(self):
        self._pending_local_motion = None
        if self._udp_receiver is not None:
            self._udp_receiver.close()


class VRMotionSource(MotionSourceBase):
    def __init__(self, policy: "TrackingPolicyRaw", policy_cfg: DictToClass):
        vr_cfg = getattr(policy_cfg, "motion_source")["vr"]
        self.vr_req_addr = reference_endpoint_from_env(
            explicit_env="G1_REF_REQ_ADDR",
            default=str(vr_cfg["req_addr"]),
            port_env="G1_REF_REQ_PORT",
        )
        self.vr_rep_addr = reference_endpoint_from_env(
            explicit_env="G1_REF_POSE_ADDR",
            default=str(vr_cfg["rep_addr"]),
            port_env="G1_REF_POSE_PORT",
        )
        self.vr_ctrl_addr = reference_endpoint_from_env(
            explicit_env="G1_REF_CTRL_ADDR",
            default=str(vr_cfg["ctrl_addr"]),
            port_env="G1_REF_CTRL_PORT",
        )
        self.vr_low_watermark = int(vr_cfg["low_watermark"])
        self.vr_high_watermark = int(vr_cfg["high_watermark"])
        self.vr_inflight_lifetime_steps = int(vr_cfg["inflight_lifetime_steps"])
        self.vr_start_button = str(
            vr_cfg.get("start_button", "right_key_one")
        ).strip()
        self.vr_stop_button = str(
            vr_cfg.get("stop_button", "left_key_one")
        ).strip()
        if (
            self.vr_start_button
            and self.vr_start_button == self.vr_stop_button
        ):
            raise ValueError("VR start_button and stop_button must differ")
        if self.vr_inflight_lifetime_steps < 0:
            raise ValueError("vr_inflight_lifetime_steps must be >= 0")
        if self.vr_high_watermark > 0 and self.vr_high_watermark < self.vr_low_watermark:
            raise ValueError("vr_high_watermark must be >= vr_low_watermark when enabled")

        self._vr_active = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        # Start-time anchor of deploy reference stream, used as transition start pose.
        self._vr_anchor_joint_pos: Optional[np.ndarray] = None
        self._vr_anchor_root_pos: Optional[np.ndarray] = None
        self._vr_anchor_root_quat: Optional[np.ndarray] = None
        self._vr_align_ready = False
        # Yaw-only alignment rotation: source(VR at start) -> target(deploy anchor at start).
        self._vr_r_delta: Optional[R] = None
        # Source VR root position at start; later VR root translation is measured relative to this origin.
        self._vr_source_root_pos0: Optional[np.ndarray] = None
        self._vr_target_anchor_pos: Optional[np.ndarray] = None

        positive_steps = [int(s) for s in np.asarray(policy.future_steps).reshape(-1).tolist() if int(s) > 0]
        self._target_future_horizon = int(max(positive_steps)) if len(positive_steps) > 0 else 0

        self._zmq_ctx = None
        self._req_sock = None
        self._rep_sock = None
        self._ctrl_sock = None
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._pending_start_request = False
        self._vr_user_enabled = False
        self._prev_start_btn = False
        self._prev_stop_btn = False
        self.remote_reference_source = ""
        self.remote_motion_name = ""
        self.remote_motion_finished = False
        self._last_motion_command_seq = -1
        self._pending_motion_command_seq: Optional[int] = None
        self._latest_control_sticks: dict[str, float] = {}
        shared_store = getattr(policy_cfg, "_pico_store", None)
        if shared_store is not None and not isinstance(shared_store, PicoFrameStore):
            raise TypeError("_pico_store must be a PicoFrameStore")
        self._shared_store: PicoFrameStore | None = shared_store
        self._hand_control_cfg = dict(getattr(policy.controller.config, "hand_control", {}))
        self._vr_stats_interval_s = 1.0
        self._vr_stats_last_monotonic = time.monotonic()
        self._vr_stats = self._new_vr_stats()

        super().__init__(policy, policy_cfg)

        if zmq is None:
            raise ImportError("[VRMotionSource] pyzmq is required for motion_source='vr'.")
        try:
            self._zmq_ctx = zmq.Context.instance()
            self._req_sock = self._zmq_ctx.socket(zmq.PUSH)
            self._req_sock.setsockopt(zmq.LINGER, 0)
            self._req_sock.setsockopt(zmq.SNDHWM, 100)
            self._req_sock.connect(self.vr_req_addr)

            self._rep_sock = self._zmq_ctx.socket(zmq.PULL)
            self._rep_sock.setsockopt(zmq.LINGER, 0)
            self._rep_sock.setsockopt(zmq.RCVHWM, 200)
            self._rep_sock.connect(self.vr_rep_addr)

            self._ctrl_sock = self._zmq_ctx.socket(zmq.PULL)
            self._ctrl_sock.setsockopt(zmq.LINGER, 0)
            self._ctrl_sock.setsockopt(zmq.RCVHWM, 200)
            self._ctrl_sock.connect(self.vr_ctrl_addr)

            print(
                "[VRMotionSource] Connected "
                f"req->{self.vr_req_addr}, rep<-{self.vr_rep_addr}, "
                f"ctrl<-{self.vr_ctrl_addr}, low_watermark={self.vr_low_watermark}, "
                f"inflight_lifetime_steps={self.vr_inflight_lifetime_steps}"
            )
        except Exception as e:
            self._req_sock = None
            self._rep_sock = None
            self._ctrl_sock = None
            print(f"[VRMotionSource] Failed to create ZMQ sockets: {e}")

    @staticmethod
    def _new_vr_stats() -> dict[str, int | None]:
        return {
            "req": 0,
            "rep": 0,
            "rep_frames": 0,
            "append": 0,
            "append_frames": 0,
            "pad": 0,
            "pad_frames": 0,
            "drop_full": 0,
            "drop_excess": 0,
            "drop_frames": 0,
            "horizon_low": 0,
            "horizon_min": None,
            "ignore_non_start": 0,
            "drop_delayed_start": 0,
            "ignore_inactive": 0,
            "ignore_no_aligned": 0,
        }

    def _bump_vr_stat(self, key: str, value: int = 1) -> None:
        self._vr_stats[key] = int(self._vr_stats[key] or 0) + int(value)

    def _record_low_horizon(self, horizon: int) -> None:
        self._bump_vr_stat("horizon_low")
        prev = self._vr_stats.get("horizon_min")
        if prev is None or int(horizon) < int(prev):
            self._vr_stats["horizon_min"] = int(horizon)

    def _print_vr_stats_if_due(self) -> None:
        now = time.monotonic()
        if (now - self._vr_stats_last_monotonic) < self._vr_stats_interval_s:
            return
        self._vr_stats_last_monotonic = now

        stats = self._vr_stats
        if not any(int(v) for v in stats.values() if v is not None):
            return

        horizon_min = stats["horizon_min"]
        horizon_msg = "None" if horizon_min is None else str(int(horizon_min))
        print(
            "[VRMotionSource][Stats] "
            f"req={int(stats['req'])}, rep={int(stats['rep'])}, "
            f"rep_frames={int(stats['rep_frames'])}, "
            f"append={int(stats['append'])}, append_frames={int(stats['append_frames'])}, "
            f"pad={int(stats['pad'])}, pad_frames={int(stats['pad_frames'])}, "
            f"drop_full={int(stats['drop_full'])}, drop_excess={int(stats['drop_excess'])}, "
            f"drop_frames={int(stats['drop_frames'])}, "
            f"horizon_low={int(stats['horizon_low'])}, horizon_min={horizon_msg}, "
            f"ignore_non_start={int(stats['ignore_non_start'])}, "
            f"drop_delayed_start={int(stats['drop_delayed_start'])}, "
            f"ignore_inactive={int(stats['ignore_inactive'])}, "
            f"ignore_no_aligned={int(stats['ignore_no_aligned'])}"
        )
        self._vr_stats = self._new_vr_stats()

    @staticmethod
    def _extract_buttons(payload: dict) -> Optional[dict]:
        if not isinstance(payload, dict):
            return None
        buttons = payload.get("controller_buttons", None)
        if not isinstance(buttons, dict):
            return None
        return buttons

    @staticmethod
    def _extract_sticks(payload: dict) -> Optional[dict]:
        buttons = VRMotionSource._extract_buttons(payload)
        if buttons is None:
            return None

        def axis_xy(key: str) -> Optional[tuple[float, float]]:
            axis = buttons.get(key, None)
            if not isinstance(axis, (list, tuple)) or len(axis) < 2:
                return None
            try:
                return float(axis[0]), float(axis[1])
            except (TypeError, ValueError):
                return None

        out = {}
        left = axis_xy("left_axis")
        right = axis_xy("right_axis")
        if left is not None:
            out["lx"], out["ly"] = left
        if right is not None:
            out["rx"], out["ry"] = right
        return out if out else None

    def _update_hand_from_sticks(self) -> None:
        if not bool(self._hand_control_cfg.get("enabled", False)):
            return
        ctrl = self.policy.controller
        if "hand_enable" not in getattr(ctrl, "extra_command", {}):
            return

        deadband = float(self._hand_control_cfg.get("deadband", 0.1))
        rate = float(self._hand_control_cfg.get("rate", 1.0))
        up_opens = bool(self._hand_control_cfg.get("up_opens", True))
        left_name = str(self._hand_control_cfg.get("left_stick", "ly"))
        right_name = str(self._hand_control_cfg.get("right_stick", "ry"))

        def axis_value(name: str) -> float:
            try:
                value = float(self._latest_control_sticks.get(name, 0.0))
            except (TypeError, ValueError):
                value = 0.0
            return float(np.clip(value, -1.0, 1.0))

        def axis_to_delta(axis: float) -> float:
            if abs(axis) < deadband:
                return 0.0
            signed = -axis if up_opens else axis
            return signed * rate * float(ctrl.control_dt)

        ctrl.set_hand_command(
            left=ctrl.hand_left + axis_to_delta(axis_value(left_name)),
            right=ctrl.hand_right + axis_to_delta(axis_value(right_name)),
            enable=1,
        )

    def request_start(self) -> None:
        """Start a fresh aligned VR reference session."""
        self._vr_user_enabled = True
        self._pending_start_request = True
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_active = False
        self._vr_align_ready = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        self.remote_motion_finished = False
        if self._shared_store is not None:
            self._shared_store.publish_control(active=False)
        print("[VRMotionSource] VR start requested")

    def request_stop(self) -> None:
        """Stop requesting and consuming VR reference frames."""
        self._vr_user_enabled = False
        self._pending_start_request = False
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_active = False
        self._vr_align_ready = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        if self._shared_store is not None:
            self._shared_store.publish_control(active=False)
        print("[VRMotionSource] VR stop requested")

    def _record_motion_command(self, payload: dict) -> bool:
        """Queue a newly selected host motion without emulating controller input."""
        if not isinstance(payload, dict):
            return False
        if str(payload.get("source", "")).strip().lower() != "motion":
            return False
        if str(payload.get("state", "")).strip().lower() != "queued":
            return False
        try:
            command_seq = int(payload.get("motion_command_seq", 0))
        except (TypeError, ValueError):
            return False
        if command_seq <= 0 or command_seq == self._last_motion_command_seq:
            return False

        self._last_motion_command_seq = command_seq
        self._pending_motion_command_seq = command_seq
        self.remote_reference_source = "motion"
        self.remote_motion_name = str(payload.get("motion", "")).strip()
        self.remote_motion_finished = False
        print(
            f"[VRMotionSource] queued host motion: "
            f"{self.remote_motion_name or '<unnamed>'} (command_seq={command_seq})"
        )
        return True

    def _start_queued_motion_if_ready(self) -> bool:
        if self._pending_motion_command_seq is None:
            return False
        if not self.policy.current_done:
            return False
        if self._vr_active or self._pending_start_request:
            return False

        command_seq = self._pending_motion_command_seq
        self._pending_motion_command_seq = None
        self.request_start()
        print(
            f"[VRMotionSource] starting queued host motion "
            f"(command_seq={command_seq})"
        )
        return True

    def _drain_control(self) -> None:
        if self._ctrl_sock is None:
            return

        latest_buttons: Optional[dict] = None
        latest_sticks: Optional[dict] = None
        latest_motion_payload: Optional[dict] = None
        pressed_buttons: set[str] = set()
        while True:
            try:
                raw = self._ctrl_sock.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception as e:
                print(f"[VRMotionSource] control recv failed: {e}")
                break

            try:
                payload = json.loads(raw)
            except Exception:
                continue
            if str(payload.get("source", "")).strip().lower() == "motion":
                latest_motion_payload = payload
            buttons = self._extract_buttons(payload)
            if buttons is not None:
                latest_buttons = buttons
                pressed_buttons.update(
                    str(name)
                    for name, value in buttons.items()
                    if isinstance(value, (bool, np.bool_)) and bool(value)
                )
            sticks = self._extract_sticks(payload)
            if sticks is not None:
                latest_sticks = sticks

        if latest_motion_payload is not None:
            self._record_motion_command(latest_motion_payload)

        if latest_sticks is not None:
            self._latest_control_sticks = {str(k): float(v) for k, v in latest_sticks.items()}
            self._update_hand_from_sticks()

        if latest_buttons is not None and pressed_buttons:
            latest_buttons = dict(latest_buttons)
            for name in pressed_buttons:
                latest_buttons[name] = True

        if self._shared_store is not None and (
            latest_buttons is not None or latest_sticks is not None
        ):
            self._shared_store.publish_control(
                buttons=latest_buttons,
                sticks=latest_sticks,
                active=self._vr_active,
            )

        if latest_buttons is None:
            return

        start_btn = bool(
            self.vr_start_button
            and latest_buttons.get(self.vr_start_button, False)
        )
        stop_btn = bool(
            self.vr_stop_button
            and latest_buttons.get(self.vr_stop_button, False)
        )
        start_rise = start_btn and (not self._prev_start_btn)
        stop_rise = stop_btn and (not self._prev_stop_btn)
        self._prev_start_btn = start_btn
        self._prev_stop_btn = stop_btn

        if stop_rise:
            self.request_stop()

        if start_rise:
            self.request_start()

    def poll_control(self) -> None:
        """Drain controller input before the task computes its next command."""
        self._drain_control()

    def _future_horizon(self) -> int:
        if self.policy.ref_len <= 0:
            return 0
        return max(0, int(self.policy.ref_len - 1 - self.policy.ref_idx))

    @staticmethod
    def _repeat_frame(frame: Dict[str, np.ndarray], count: int) -> Dict[str, np.ndarray]:
        c = int(count)
        return {
            "joint_pos": np.repeat(frame["joint_pos"].reshape(1, -1), c, axis=0).astype(np.float32),
            "root_pos": np.repeat(frame["root_pos"].reshape(1, -1), c, axis=0).astype(np.float32),
            "root_quat": np.repeat(frame["root_quat"].reshape(1, -1), c, axis=0).astype(np.float32),
        }

    def _pad_future_once_on_start(self, frame: Dict[str, np.ndarray]) -> None:
        if self._target_future_horizon <= 0:
            return
        deficit = int(self._target_future_horizon - self._future_horizon())
        if deficit > 0:
            self.policy.append_ref_frames(self._repeat_frame(frame, deficit))

    def _pad_future_to_low_watermark(self, frame: Dict[str, np.ndarray]) -> None:
        deficit = int(self.vr_low_watermark - self._future_horizon())
        if deficit <= 0:
            return
        self.policy.append_ref_frames(self._repeat_frame(frame, deficit))
        self._bump_vr_stat("pad")
        self._bump_vr_stat("pad_frames", deficit)

    def _appendable_reply_frames(self, frames: list[Dict[str, np.ndarray]]) -> list[Dict[str, np.ndarray]]:
        if self.vr_high_watermark <= 0:
            return frames
        h_now = self._future_horizon()
        if h_now >= self.vr_high_watermark:
            self._bump_vr_stat("drop_full")
            self._bump_vr_stat("drop_frames", len(frames))
            return []
        capacity = int(self.vr_high_watermark - h_now)
        kept = frames[:capacity]
        dropped = max(0, len(frames) - len(kept))
        if dropped > 0:
            self._bump_vr_stat("drop_excess")
            self._bump_vr_stat("drop_frames", dropped)
        return kept

    def _warn_horizon_if_needed(self, tag: str) -> None:
        if self._target_future_horizon <= 0:
            return
        if (not self._vr_user_enabled) and (not self._pending_start_request) and (not self._vr_active):
            return
        h = self._future_horizon()
        if h < self._target_future_horizon:
            self._record_low_horizon(h)

    @staticmethod
    def _slerp_single_shortest(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
        a = float(np.clip(alpha, 0.0, 1.0))
        qq0 = np.asarray(q0, dtype=np.float64).reshape(4)
        qq1 = np.asarray(q1, dtype=np.float64).reshape(4)
        qq0 /= max(np.linalg.norm(qq0), 1e-9)
        qq1 /= max(np.linalg.norm(qq1), 1e-9)
        if float(np.dot(qq0, qq1)) < 0.0:
            qq1 = -qq1
        key = R.from_quat(np.stack([qq0, qq1], axis=0), scalar_first=True)
        interp = Slerp([0.0, 1.0], key)([a]).as_quat(scalar_first=True)[0]
        interp = interp / max(np.linalg.norm(interp), 1e-9)
        return interp.astype(np.float32)

    def _parse_qpos_frames(
        self,
        payload: dict,
        qpos_payload: memoryview,
    ) -> list[Dict[str, np.ndarray]]:
        if not isinstance(payload, dict):
            return []
        try:
            num_frames = int(payload.get("num_frames", 0))
            qpos_size = int(payload.get("qpos_size", 0))
        except Exception:
            return []
        if num_frames <= 0 or qpos_size < 7:
            return []

        try:
            qpos_batch = np.frombuffer(qpos_payload, dtype=np.float32).reshape(num_frames, qpos_size)
        except Exception:
            return []

        joint_pos_batch = qpos_batch[:, 7:]
        if joint_pos_batch.shape[1] != self.policy.n_joints:
            print(
                f"[VRMotionSource] dof dim mismatch: "
                f"got={joint_pos_batch.shape[1]}, expected={self.policy.n_joints}"
            )
            return []
        if len(self.policy.dataset_joint_names) != joint_pos_batch.shape[1]:
            print(
                f"[VRMotionSource] dataset_joint_names mismatch: "
                f"got={len(self.policy.dataset_joint_names)}, expected={joint_pos_batch.shape[1]}"
            )
            return []
        joint_pos_batch = remap_joint_array_by_names(
            joint_pos_batch,
            self.policy.dataset_joint_names,
            self.policy.obs_joint_names,
        )

        parsed_frames: list[Dict[str, np.ndarray]] = []
        for frame_idx in range(num_frames):
            root_pos = qpos_batch[frame_idx, 0:3].astype(np.float32, copy=True)
            root_quat = qpos_batch[frame_idx, 3:7].astype(np.float32, copy=True)
            qn = float(np.linalg.norm(root_quat))
            if not np.isfinite(qn) or qn < 1e-6:
                continue
            root_quat = (root_quat / qn).astype(np.float32)
            parsed_frames.append(
                {
                    "joint_pos": joint_pos_batch[frame_idx].astype(np.float32, copy=True),
                    "root_pos": root_pos,
                    "root_quat": root_quat,
                }
            )
        return parsed_frames

    def _start_vr_session(self, first_frame: Dict[str, np.ndarray]) -> None:
        anchor = self.policy.read_ref_tail_state()
        self._vr_anchor_joint_pos = anchor["joint_pos"].astype(np.float32, copy=True)
        self._vr_anchor_root_pos = anchor["root_pos"].astype(np.float32, copy=True)
        self._vr_anchor_root_quat = anchor["root_quat"].astype(np.float32, copy=True)
        src_yaw = _yaw_component_wxyz(first_frame["root_quat"])
        tgt_yaw = _yaw_component_wxyz(anchor["root_quat"])
        r0 = R.from_quat(src_yaw, scalar_first=True)
        rc = R.from_quat(tgt_yaw, scalar_first=True)
        self._vr_r_delta = rc * r0.inv()
        self._vr_source_root_pos0 = first_frame["root_pos"].astype(np.float32, copy=True)
        self._vr_target_anchor_pos = anchor["root_pos"].astype(np.float32, copy=True)
        self._vr_align_ready = True
        self._vr_active = True
        self._vr_transition_count = 0
        self._vr_in_transition = int(self.policy.transition_steps) > 0
        self._pending_start_request = False
        # Bootstrap future horizon once at start using current ref-buffer tail.
        self._pad_future_once_on_start(
            {
                "joint_pos": anchor["joint_pos"].astype(np.float32, copy=True),
                "root_pos": anchor["root_pos"].astype(np.float32, copy=True),
                "root_quat": anchor["root_quat"].astype(np.float32, copy=True),
            }
        )
        print(
            "[VRMotionSource] VR start acknowledged "
            f"(transition_steps={int(self.policy.transition_steps)})"
        )

    def _apply_start_transition(self, aligned: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        if (
            not self._vr_in_transition
            or self._vr_anchor_joint_pos is None
            or self._vr_anchor_root_pos is None
            or self._vr_anchor_root_quat is None
        ):
            return aligned

        self._vr_transition_count += 1
        t_steps = max(1, int(self.policy.transition_steps))
        alpha = min(1.0, float(self._vr_transition_count) / float(t_steps))

        out_joint = (self._vr_anchor_joint_pos * (1.0 - alpha) + aligned["joint_pos"] * alpha).astype(np.float32)
        out_pos = (self._vr_anchor_root_pos * (1.0 - alpha) + aligned["root_pos"] * alpha).astype(np.float32)
        out_quat = self._slerp_single_shortest(self._vr_anchor_root_quat, aligned["root_quat"], alpha)

        if alpha >= 1.0:
            self._vr_in_transition = False
        return {
            "joint_pos": out_joint,
            "root_pos": out_pos,
            "root_quat": out_quat,
        }

    def _align_vr_frame(self, frame: Dict[str, np.ndarray]) -> Optional[Dict[str, np.ndarray]]:
        if (
            not self._vr_align_ready
            or self._vr_r_delta is None
            or self._vr_source_root_pos0 is None
            or self._vr_target_anchor_pos is None
        ):
            return None
        root_pos = frame["root_pos"].astype(np.float32)
        root_quat = frame["root_quat"].astype(np.float32)

        aligned_pos = self._vr_r_delta.apply(root_pos - self._vr_source_root_pos0) + self._vr_target_anchor_pos
        aligned_pos = aligned_pos.astype(np.float32)
        aligned_pos[2] = root_pos[2]

        aligned_quat = (self._vr_r_delta * R.from_quat(root_quat, scalar_first=True)).as_quat(scalar_first=True)
        aligned_quat = aligned_quat.astype(np.float32)
        aligned_quat /= max(np.linalg.norm(aligned_quat), 1e-6)
        return {
            "joint_pos": frame["joint_pos"].astype(np.float32, copy=True),
            "root_pos": aligned_pos,
            "root_quat": aligned_quat,
        }

    def _drain_replies(self) -> Optional[Dict[str, np.ndarray]]:
        last_aligned_frame: Optional[Dict[str, np.ndarray]] = None
        if self._rep_sock is None:
            return last_aligned_frame
        while True:
            try:
                parts = self._rep_sock.recv_multipart(flags=zmq.NOBLOCK, copy=False)
            except zmq.Again:
                break
            except Exception as e:
                print(f"[VRMotionSource] recv failed: {e}")
                break

            if len(parts) != 2:
                print(f"[VRMotionSource] bad multipart reply: expected 2 parts, got {len(parts)}")
                continue
            try:
                payload = json.loads(bytes(parts[0]))
            except Exception:
                print("[VRMotionSource] bad reply header")
                continue
            if not isinstance(payload, dict):
                continue
            protocol = payload.get("protocol")
            if protocol is not None and protocol != "g1-reference-v1":
                print(f"[VRMotionSource] unsupported reference protocol: {protocol!r}")
                continue
            reply_source = str(payload.get("source", "")).strip().lower()
            reply_motion_name = str(payload.get("motion", "")).strip()
            reply_finished = bool(payload.get("finished", False))

            parsed_frames = self._parse_qpos_frames(payload, parts[1].buffer)
            if len(parsed_frames) == 0:
                continue

            self._bump_vr_stat("rep")
            self._bump_vr_stat("rep_frames", len(parsed_frames))

            start_flag = bool(payload.get("start", False))
            if self._pending_start_request and not start_flag:
                self._bump_vr_stat("ignore_non_start")
                continue
            if start_flag:
                if self._pending_start_request:
                    self._start_vr_session(parsed_frames[0])
                else:
                    self._bump_vr_stat("drop_delayed_start")
                    continue

            if not self._vr_active:
                self._bump_vr_stat("ignore_inactive")
                continue

            if reply_source:
                self.remote_reference_source = reply_source
            if reply_motion_name:
                self.remote_motion_name = reply_motion_name

            out_frames = []
            for f in parsed_frames:
                aligned = self._align_vr_frame(f)
                if aligned is not None:
                    out_frames.append(self._apply_start_transition(aligned))
            out_frames = self._appendable_reply_frames(out_frames)
            if len(out_frames) == 0:
                self._bump_vr_stat("ignore_no_aligned")
                if reply_source == "motion" and reply_finished:
                    self._finish_remote_motion()
                continue

            seg = {
                "joint_pos": np.stack([f["joint_pos"] for f in out_frames], axis=0).astype(np.float32),
                "root_pos": np.stack([f["root_pos"] for f in out_frames], axis=0).astype(np.float32),
                "root_quat": np.stack([f["root_quat"] for f in out_frames], axis=0).astype(np.float32),
            }
            self.policy.append_ref_frames(seg)
            last_aligned_frame = out_frames[-1]
            if reply_source == "motion" and reply_finished:
                self._finish_remote_motion()
            if self._shared_store is not None:
                self._shared_store.publish_frame(
                    last_aligned_frame,
                    joint_names=self.policy.obs_joint_names,
                    active=self._vr_active,
                )
            self._bump_vr_stat("append")
            self._bump_vr_stat("append_frames", len(out_frames))
        return last_aligned_frame

    def _finish_remote_motion(self) -> None:
        """Keep policy control active and append a local default reference."""
        self.remote_motion_finished = True
        self._vr_user_enabled = False
        self._pending_start_request = False
        self._vr_active = False
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_in_transition = False
        self._vr_transition_count = 0
        if self._shared_store is not None:
            self._shared_store.publish_control(active=False)
        if not self.append_motion_from_tail("default"):
            print("[VRMotionSource] failed to append the default reference")
        print(
            f"[VRMotionSource] motion finished: "
            f"{self.remote_motion_name or '<unnamed>'}; returning to default reference"
        )

    def _send_request_if_needed(self) -> None:
        if self._req_sock is None:
            return
        if not self._vr_user_enabled:
            return
        if self._req_inflight:
            return
        h = self._future_horizon()
        should_request = (h <= self.vr_low_watermark) or self._pending_start_request
        if not should_request:
            return
        start_flag = bool(self._pending_start_request)
        req = {"start": start_flag}
        try:
            self._req_sock.send_string(json.dumps(req), flags=zmq.NOBLOCK)
            self._req_inflight = True
            self._req_inflight_steps_left = int(self.vr_inflight_lifetime_steps)
            self._bump_vr_stat("req")
        except zmq.Again:
            return
        except Exception as e:
            print(f"[VRMotionSource] send request failed: {e}")

    def on_fade_in(self):
        self.append_motion_from_tail("default")
        self._last_motion_command_seq = -1
        self._pending_motion_command_seq = None
        self._pending_start_request = False
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_user_enabled = False
        self._prev_start_btn = False
        self._prev_stop_btn = False
        self._vr_active = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        self._vr_anchor_joint_pos = None
        self._vr_anchor_root_pos = None
        self._vr_anchor_root_quat = None
        self._vr_align_ready = False
        if self._shared_store is not None:
            self._shared_store.publish_control(active=False)

    def on_fade_out(self):
        self._vr_user_enabled = False
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_active = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        self._vr_anchor_joint_pos = None
        self._vr_anchor_root_pos = None
        self._vr_anchor_root_quat = None
        self._vr_align_ready = False
        self._pending_start_request = False
        self._pending_motion_command_seq = None
        super().on_fade_out()

    def post_step(self):
        self.poll_control()
        last_aligned_frame = self._drain_replies()
        if last_aligned_frame is not None:
            self._req_inflight = False
            self._req_inflight_steps_left = 0
            if self._vr_active:
                self._pad_future_to_low_watermark(last_aligned_frame)
        elif self._req_inflight:
            self._req_inflight_steps_left -= 1
            if self._req_inflight_steps_left <= 0:
                self._req_inflight = False
                self._req_inflight_steps_left = 0
        self._start_queued_motion_if_ready()
        self._send_request_if_needed()
        self._warn_horizon_if_needed("post_step")
        self._print_vr_stats_if_due()

    def deactivate(self):
        self._vr_user_enabled = False
        self._req_inflight = False
        self._req_inflight_steps_left = 0
        self._vr_active = False
        self._vr_in_transition = False
        self._vr_transition_count = 0
        self._vr_anchor_joint_pos = None
        self._vr_anchor_root_pos = None
        self._vr_anchor_root_quat = None
        self._vr_align_ready = False
        self._pending_start_request = False
        self._pending_motion_command_seq = None
        if self._shared_store is not None:
            self._shared_store.publish_control(active=False)
        if self._req_sock is not None:
            try:
                self._req_sock.close(0)
            except Exception:
                pass
            self._req_sock = None
        if self._rep_sock is not None:
            try:
                self._rep_sock.close(0)
            except Exception:
                pass
            self._rep_sock = None
        if self._ctrl_sock is not None:
            try:
                self._ctrl_sock.close(0)
            except Exception:
                pass
            self._ctrl_sock = None
