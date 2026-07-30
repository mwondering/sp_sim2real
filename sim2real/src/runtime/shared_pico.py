from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class PicoSnapshot:
    """Immutable latest-value snapshot shared by the two task policies."""

    seq: int
    timestamp: float
    pose_timestamp: float
    control_timestamp: float
    joint_names: tuple[str, ...]
    joint_pos: np.ndarray | None
    root_pos: np.ndarray | None
    root_quat: np.ndarray | None
    buttons: dict[str, object]
    sticks: dict[str, float]
    active: bool

    @property
    def has_pose(self) -> bool:
        return (
            self.joint_pos is not None
            and self.root_pos is not None
            and self.root_quat is not None
        )


class PicoFrameStore:
    """Thread-safe single-writer/latest-reader store.

    The PICO ZMQ sockets remain owned by ``VRMotionSource``.  This store only
    mirrors already aligned frames and controller input, so multiple policies
    can read them without consuming or competing for transport messages.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._seq = 0
        self._timestamp = 0.0
        self._pose_timestamp = 0.0
        self._control_timestamp = 0.0
        self._joint_names: tuple[str, ...] = ()
        self._joint_pos: np.ndarray | None = None
        self._root_pos: np.ndarray | None = None
        self._root_quat: np.ndarray | None = None
        self._buttons: dict[str, object] = {}
        self._sticks: dict[str, float] = {}
        self._active = False

    def publish_control(
        self,
        *,
        buttons: Mapping[str, object] | None = None,
        sticks: Mapping[str, float] | None = None,
        active: bool | None = None,
    ) -> None:
        with self._lock:
            if buttons is not None:
                self._buttons = dict(buttons)
            if sticks is not None:
                self._sticks = {str(k): float(v) for k, v in sticks.items()}
            if active is not None:
                self._active = bool(active)
            self._seq += 1
            self._timestamp = time.monotonic()
            self._control_timestamp = self._timestamp

    def publish_frame(
        self,
        frame: Mapping[str, np.ndarray],
        *,
        joint_names: Sequence[str],
        active: bool,
    ) -> None:
        joint_pos = np.asarray(frame["joint_pos"], dtype=np.float32).reshape(-1)
        root_pos = np.asarray(frame["root_pos"], dtype=np.float32).reshape(3)
        root_quat = np.asarray(frame["root_quat"], dtype=np.float32).reshape(4)
        names = tuple(str(name) for name in joint_names)
        if joint_pos.shape != (len(names),):
            raise ValueError(
                f"PICO frame has {joint_pos.size} joints but {len(names)} names"
            )
        with self._lock:
            self._joint_names = names
            self._joint_pos = joint_pos.copy()
            self._root_pos = root_pos.copy()
            self._root_quat = root_quat.copy()
            self._active = bool(active)
            self._seq += 1
            self._timestamp = time.monotonic()
            self._pose_timestamp = self._timestamp

    def snapshot(self) -> PicoSnapshot:
        with self._lock:
            return PicoSnapshot(
                seq=self._seq,
                timestamp=self._timestamp,
                pose_timestamp=self._pose_timestamp,
                control_timestamp=self._control_timestamp,
                joint_names=self._joint_names,
                joint_pos=None if self._joint_pos is None else self._joint_pos.copy(),
                root_pos=None if self._root_pos is None else self._root_pos.copy(),
                root_quat=None if self._root_quat is None else self._root_quat.copy(),
                buttons=dict(self._buttons),
                sticks=dict(self._sticks),
                active=self._active,
            )


__all__ = ["PicoFrameStore", "PicoSnapshot"]
