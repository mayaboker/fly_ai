"""Small bridge for the Godot renderer used by the ``godot-render`` branch."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
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


@dataclass(frozen=True)
class FrameSample:
    """One stable shared-memory frame and its producer sequence number."""

    image: np.ndarray
    sequence: int


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
        self._pending_collision: str | None = None
        self._performance_metrics: dict[str, object] | None = None
        self._control_events: deque[str] = deque()

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

    def publish_control_state(self, enabled: bool, running: bool, completed: bool = False) -> None:
        """Tell Godot whether to show controls and which actions are valid."""
        self._send({"controls": {"enabled": enabled, "running": running, "completed": completed}})

    def publish_render_settings(self, capture_hz: int) -> None:
        """Configure renderer-owned behavior from the authoritative scenario."""
        self._send({"render_settings": {"capture_hz": capture_hz}})

    def _send(self, payload: dict[str, object]) -> None:
        """Encode and send one compact renderer packet."""
        self._socket.sendto(json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8"), self.destination)

    def read_frame(self) -> np.ndarray | None:
        """Return one RGB frame, or ``None`` while Godot is writing a frame."""
        sample = self.read_frame_sample()
        return sample.image if sample else None

    def read_frame_sample(self) -> FrameSample | None:
        """Return one stable RGB frame with sequence metadata."""
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
        image = np.frombuffer(data, dtype=np.uint8).reshape(height, width, channels).copy()
        return FrameSample(image, sequence)

    def read_collision_event(self) -> str | None:
        """Return Godot's newest target or obstacle collision event, if any."""
        self._drain_events()
        event, self._pending_collision = self._pending_collision, None
        return event

    def read_performance_metrics(self) -> dict[str, object] | None:
        """Return the newest renderer counters received from Godot."""
        self._drain_events()
        metrics, self._performance_metrics = self._performance_metrics, None
        return metrics

    def read_control_events(self) -> list[str]:
        """Drain ordered interactive commands received from Godot."""
        self._drain_events()
        commands = list(self._control_events)
        self._control_events.clear()
        return commands

    def _drain_events(self) -> None:
        """Route all queued Godot event packets without losing either kind."""
        while True:
            try:
                payload, _ = self._event_socket.recvfrom(1024)
            except BlockingIOError:
                return
            try:
                value = json.loads(payload)
            except (TypeError, json.JSONDecodeError):
                continue
            kind = value.get("kind") if isinstance(value, dict) else None
            if isinstance(value, dict) and value.get("event") == "collision" and kind in {"target", "obstacle"}:
                self._pending_collision = kind
            elif isinstance(value, dict) and value.get("event") == "performance" and isinstance(value.get("metrics"), dict):
                self._performance_metrics = value["metrics"]
            elif isinstance(value, dict) and value.get("event") == "control" and value.get("command") in {"start", "pause", "restart", "stop"}:
                self._control_events.append(value["command"])

    def clear_collision_events(self) -> None:
        """Discard stale Godot collision events before a restarted attempt."""
        self._drain_events()
        self._pending_collision = None

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
