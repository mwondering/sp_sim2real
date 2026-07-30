from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Mapping

import numpy as np

try:
    import zmq
except Exception:
    zmq = None


def _require_zmq() -> None:
    if zmq is None:
        raise ImportError("pyzmq is required for the sim-state/depth streams")


class ArrayPublisher:
    """Non-blocking latest-value PUB socket for one float32 array."""

    def __init__(self, bind: str, *, topic: str) -> None:
        _require_zmq()
        self.topic = str(topic).encode("utf-8")
        self.socket = zmq.Context.instance().socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 2)
        self.socket.bind(str(bind))

    def send(
        self,
        values: np.ndarray,
        *,
        seq: int,
        sim_time: float,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        array = np.ascontiguousarray(values, dtype=np.float32)
        header = {
            "seq": int(seq),
            "sim_time": float(sim_time),
            "wall_time": time.time(),
            "shape": list(array.shape),
            "dtype": "float32",
        }
        if metadata:
            header.update(dict(metadata))
        try:
            self.socket.send_multipart(
                [
                    self.topic,
                    json.dumps(header, separators=(",", ":")).encode("utf-8"),
                    memoryview(array),
                ],
                flags=zmq.NOBLOCK,
                copy=False,
            )
            return True
        except zmq.Again:
            return False

    def close(self) -> None:
        self.socket.close(0)


@dataclass(frozen=True)
class ArrayPacket:
    seq: int
    sim_time: float
    recv_time: float
    values: np.ndarray
    metadata: dict[str, object]


class ArraySubscriber:
    """Latest-value SUB socket; old frames are drained on every read."""

    def __init__(self, connect: str, *, topic: str) -> None:
        _require_zmq()
        self.topic = str(topic).encode("utf-8")
        self.socket = zmq.Context.instance().socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVHWM, 2)
        self.socket.setsockopt(zmq.CONFLATE, 0)
        self.socket.setsockopt(zmq.SUBSCRIBE, self.topic)
        self.socket.connect(str(connect))
        self._latest: ArrayPacket | None = None

    def read_latest(self) -> ArrayPacket | None:
        while True:
            try:
                parts = self.socket.recv_multipart(flags=zmq.NOBLOCK, copy=False)
            except zmq.Again:
                break
            if len(parts) != 3 or bytes(parts[0]) != self.topic:
                continue
            try:
                header = json.loads(bytes(parts[1]))
                shape = tuple(int(v) for v in header["shape"])
                values = (
                    np.frombuffer(parts[2].buffer, dtype=np.float32)
                    .reshape(shape)
                    .copy()
                )
                self._latest = ArrayPacket(
                    seq=int(header["seq"]),
                    sim_time=float(header["sim_time"]),
                    recv_time=time.monotonic(),
                    values=values,
                    metadata=header,
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
        return self._latest

    def close(self) -> None:
        self.socket.close(0)


__all__ = ["ArrayPacket", "ArrayPublisher", "ArraySubscriber"]
