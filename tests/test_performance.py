"""Tests for opt-in simulation performance artifacts."""

import csv
import json

import pytest

from fly_smart.simulation.performance import PerformanceProfiler
from fly_smart.simulation.pacing import PacingResult


class _Clock:
    """Deterministic monotonic nanosecond clock for profiler tests."""

    def __init__(self) -> None:
        self.value = 0

    def __call__(self) -> int:
        return self.value


def test_profiler_writes_artifacts_and_counts_frame_sequences(tmp_path):
    clock = _Clock()
    profiler = PerformanceProfiler(tmp_path, physics_hz=2, camera_hz=1, clock_ns=clock, echo=False)
    profiler.begin_step(0, 0.0)
    with profiler.measure("physics"):
        clock.value += 2_000_000
    assert profiler.camera_frame(2) == "new"
    assert profiler.camera_frame(2) == "duplicate"
    assert profiler.camera_frame(8) == "new"
    assert profiler.camera_frame(None) == "missing"
    clock.value = 4_000_000
    profiler.end_step("new", PacingResult(0.001, 0.0011, 0.0001, True))
    clock.value = 5_000_000
    summary = profiler.finish(0.5)

    assert summary["real_time_factor"] == 100.0
    assert summary["camera_frames"] == {"new": 2, "duplicate": 1, "missing": 1, "skipped": 2}
    assert summary["ranked_stages"][0]["stage"] == "physics"
    with (tmp_path / "performance.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["physics_ms"] == "2.0"
    assert rows[0]["pacing_rebased"] == "True"
    assert float(rows[0]["deadline_lateness_ms"]) == pytest.approx(0.1, abs=1e-9)
    trace = json.loads((tmp_path / "performance-trace.json").read_text())
    assert trace["traceEvents"][0]["name"] == "physics"
    assert (tmp_path / "performance-counters.csv").is_file()
