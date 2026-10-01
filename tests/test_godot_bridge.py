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


class _EventSocket:
    """Return queued datagrams and then behave like a nonblocking socket."""

    def __init__(self, values):
        self.values = list(values)

    def recvfrom(self, _size):
        if not self.values:
            raise BlockingIOError
        return json.dumps(self.values.pop(0)).encode(), ("127.0.0.1", 9101)


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


def test_publish_control_state_uses_renderer_channel():
    bridge = GodotBridge.__new__(GodotBridge)
    bridge.destination = ("127.0.0.1", 9100)
    bridge._socket = _RecordingSocket()

    bridge.publish_control_state(enabled=True, running=False)

    assert json.loads(bridge._socket.payload) == {"controls": {"enabled": True, "running": False}}


def test_performance_packet_does_not_hide_collision_event():
    bridge = GodotBridge.__new__(GodotBridge)
    bridge._event_socket = _EventSocket([
        {"event": "performance", "metrics": {"fps": 60, "capture_average_ms": 1.5}},
        {"event": "collision", "kind": "target"},
    ])
    bridge._pending_collision = None
    bridge._performance_metrics = None
    bridge._control_events = __import__("collections").deque()

    assert bridge.read_collision_event() == "target"
    assert bridge.read_performance_metrics() == {"fps": 60, "capture_average_ms": 1.5}


def test_control_packets_remain_ordered_beside_other_events():
    bridge = GodotBridge.__new__(GodotBridge)
    bridge._event_socket = _EventSocket([
        {"event": "control", "command": "start"},
        {"event": "performance", "metrics": {"fps": 50}},
        {"event": "control", "command": "restart"},
    ])
    bridge._pending_collision = None
    bridge._performance_metrics = None
    bridge._control_events = __import__("collections").deque()

    assert bridge.read_control_events() == ["start", "restart"]
    assert bridge.read_performance_metrics() == {"fps": 50}
