"""Tests for the Python-to-Godot display bridge."""

import json

from fly_smart.simulation.godot_bridge import GodotBridge, sanitize_telemetry


class _RecordingSocket:
    """Capture one UDP payload without opening a network socket."""

    def __init__(self) -> None:
        self.payload = b""
        self.destination = None

    def sendto(self, payload: bytes, destination) -> None:
        self.payload = payload
        self.destination = destination


def test_sanitize_telemetry_replaces_non_finite_values_recursively():
    telemetry = {
        "ttc_s": float("nan"),
        "growth_px_s": float("inf"),
        "position_m": (1.0, 2.0, float("-inf")),
    }

    assert sanitize_telemetry(telemetry) == {
        "ttc_s": None,
        "growth_px_s": None,
        "position_m": [1.0, 2.0, None],
    }


def test_publish_telemetry_sends_optional_display_packet():
    bridge = GodotBridge.__new__(GodotBridge)
    bridge.destination = ("127.0.0.1", 9100)
    bridge._socket = _RecordingSocket()

    bridge.publish_telemetry({"phase": "track", "bbox": (1, 2, 3, 4), "ttc_s": None})

    assert json.loads(bridge._socket.payload) == {
        "telemetry": {"phase": "track", "bbox": [1, 2, 3, 4], "ttc_s": None}
    }
    assert bridge._socket.destination == bridge.destination


def test_pose_packet_remains_valid_without_telemetry():
    bridge = GodotBridge.__new__(GodotBridge)
    bridge.destination = ("127.0.0.1", 9100)
    bridge._socket = _RecordingSocket()

    bridge.publish_pose((1, 2, 3), (0, 0, 0, 1), (4, 5, 6), (0, 0, 0, 1), reset=True)

    packet = json.loads(bridge._socket.payload)
    assert packet["drone"]["p"] == [1, 2, 3]
    assert packet["target"]["p"] == [4, 5, 6]
    assert packet["reset"] is True
    assert "telemetry" not in packet
