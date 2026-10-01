"""Small bridge for the Godot renderer used by the ``godot-render`` branch."""

from __future__ import annotations

import json
from math import isfinite
import mmap
from pathlib import Path
import socket
import struct

import numpy as np


HEADER = struct.Struct("<4s7I")
HEADER_BYTES = HEADER.size
COLLISION_PORT = 9101


def sanitize_telemetry(value):
    """Return JSON-safe telemetry with unavailable numeric values as ``None``."""
    if isinstance(value, dict):
        return {key: sanitize_telemetry(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_telemetry(item) for item in value]
    if isinstance(value, float) and not isfinite(value):
        return None
    return value


class GodotBridge:
    """Publish PyBullet poses and read the newest Godot RGB camera frame."""

    def __init__(self, path: Path = Path("/dev/shm/fly_smart_fpv.rgb"), port: int = 9100, event_port: int = COLLISION_PORT) -> None:
        self.path = path
        self.destination = ("127.0.0.1", port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._event_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._event_socket.bind(("127.0.0.1", event_port))
        self._event_socket.setblocking(False)
        self._file = None
        self._mapping = None

    def open(self) -> None:
        """Open the shared-memory-backed file created by Godot."""
        if not self.path.is_file():
            raise RuntimeError(f"Godot shared-memory file is missing: {self.path}")
        self._file = self.path.open("rb")
        self._mapping = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)

    def publish_pose(self, drone_position, drone_orientation, target_position, target_orientation, reset: bool = False) -> None:
        """Send the latest PyBullet poses; Godot drains old packets each frame."""
        payload = {
            "drone": {"p": list(drone_position), "q": list(drone_orientation)},
            "target": {"p": list(target_position), "q": list(target_orientation)},
        }
        if reset:
            payload["reset"] = True
        self._send(payload)

    def publish_telemetry(self, telemetry: dict[str, object]) -> None:
        """Publish display-only flight state without changing renderer poses."""
        self._send({"telemetry": sanitize_telemetry(telemetry)})

    def _send(self, payload: dict[str, object]) -> None:
        """Encode and send one compact renderer packet."""
        self._socket.sendto(json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8"), self.destination)

    def read_frame(self) -> np.ndarray | None:
        """Return one RGB frame, or ``None`` while Godot is writing a frame."""
        if self._mapping is None:
            return None
        magic, width, height, channels, slot, sequence, frame_bytes, _ = HEADER.unpack_from(self._mapping)
        if magic != b"GFPV" or channels != 3 or slot > 1 or sequence == 0 or sequence & 1:
            return None
        if frame_bytes != width * height * channels or len(self._mapping) < HEADER_BYTES + 2 * frame_bytes:
            return None
        start = HEADER_BYTES + slot * frame_bytes
        data = self._mapping[start : start + frame_bytes]
        if struct.unpack_from("<I", self._mapping, 20)[0] != sequence:
            return None
        return np.frombuffer(data, dtype=np.uint8).reshape(height, width, channels).copy()

    def read_collision_event(self) -> str | None:
        """Return Godot's newest target or obstacle collision event, if any."""
        event = None
        while True:
            try:
                payload, _ = self._event_socket.recvfrom(1024)
            except BlockingIOError:
                return event
            try:
                value = json.loads(payload)
            except (TypeError, json.JSONDecodeError):
                continue
            kind = value.get("kind") if isinstance(value, dict) else None
            if isinstance(value, dict) and value.get("event") == "collision" and kind in {"target", "obstacle"}:
                event = kind

    def clear_collision_events(self) -> None:
        """Discard stale Godot collision events before a restarted attempt."""
        while True:
            try:
                self._event_socket.recvfrom(1024)
            except BlockingIOError:
                return

    def close(self) -> None:
        if self._mapping is not None:
            self._mapping.close()
            self._mapping = None
        if self._file is not None:
            self._file.close()
            self._file = None
        self._socket.close()
        self._event_socket.close()

    def __enter__(self) -> "GodotBridge":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
