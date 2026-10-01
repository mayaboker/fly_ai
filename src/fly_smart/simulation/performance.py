"""Opt-in stage timing and offline performance artifacts for simulation runs."""

from __future__ import annotations

from contextlib import contextmanager
import csv
import json
from pathlib import Path
from time import perf_counter_ns
from typing import TYPE_CHECKING, Callable, Iterator

if TYPE_CHECKING:
    from .pacing import PacingResult


STAGES = (
    "state_sensors",
    "camera_pose",
    "frame_copy",
    "detection",
    "video_encode",
    "guidance_control",
    "physics",
    "telemetry_publish",
    "collision",
    "plot_display",
    "sleep",
)


def _percentile(values: list[float], percentile: float) -> float:
    """Return a linearly interpolated percentile, or zero for no samples."""
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class PerformanceProfiler:
    """Collect simulation-loop spans and write CSV, JSON, and trace outputs."""

    def __init__(
        self,
        output_dir: Path,
        physics_hz: int,
        camera_hz: int,
        clock_ns: Callable[[], int] = perf_counter_ns,
        echo: bool = True,
    ) -> None:
        self.output_dir = output_dir
        self.physics_hz = physics_hz
        self.camera_hz = camera_hz
        self._clock_ns = clock_ns
        self._echo = echo
        self._started_ns = clock_ns()
        self._running = False
        self._step_started_ns = self._started_ns
        self._step = 0
        self._sim_time_s = 0.0
        self._durations_ns: dict[str, int] = {}
        self._rows: list[dict[str, object]] = []
        self._counter_rows: list[dict[str, object]] = []
        self._trace: list[dict[str, object]] = []
        self._godot: dict[str, object] = {}
        self._last_counter_second = 0
        self._counter_row_index = 0
        self._counter_wall_s = 0.0
        self._counter_camera_new = 0
        self._last_sequence: int | None = None
        self._camera_counts = {"new": 0, "duplicate": 0, "missing": 0, "skipped": 0}
        self._closed = False

    def begin_step(self, step: int, simulated_time_s: float) -> None:
        """Start one physics-step record."""
        if not self._running:
            self._started_ns = self._clock_ns()
            self._counter_wall_s = 0.0
            self._running = True
        self._step = step
        self._sim_time_s = simulated_time_s
        self._step_started_ns = self._clock_ns()
        self._durations_ns = {}

    @contextmanager
    def measure(self, stage: str) -> Iterator[None]:
        """Accumulate one named duration and add it to the timeline trace."""
        started = self._clock_ns()
        try:
            yield
        finally:
            ended = self._clock_ns()
            duration = ended - started
            self._durations_ns[stage] = self._durations_ns.get(stage, 0) + duration
            self._trace.append({
                "name": stage,
                "cat": "simulation",
                "ph": "X",
                "pid": 1,
                "tid": 1,
                "ts": (started - self._started_ns) / 1000.0,
                "dur": duration / 1000.0,
                "args": {"step": self._step, "simulated_time_s": self._sim_time_s},
            })

    def camera_frame(self, sequence: int | None) -> str:
        """Classify shared-memory sequence continuity and update counters."""
        if sequence is None:
            status = "missing"
        elif sequence == self._last_sequence:
            status = "duplicate"
        else:
            status = "new"
            if self._last_sequence is not None and sequence > self._last_sequence + 2:
                self._camera_counts["skipped"] += (sequence - self._last_sequence) // 2 - 1
            self._last_sequence = sequence
        self._camera_counts[status] += 1
        return status

    def update_godot(self, metrics: dict[str, object] | None) -> None:
        """Keep the latest renderer counters for interval output."""
        if metrics:
            self._godot = dict(metrics)

    def end_step(self, camera_status: str = "", pacing: PacingResult | None = None) -> None:
        """Finish one row and emit a one-second aggregate when due."""
        ended = self._clock_ns()
        elapsed_s = (ended - self._started_ns) / 1e9
        total_ms = (ended - self._step_started_ns) / 1e6
        deadline_s = (self._step + 1) / self.physics_hz
        lateness_ms = pacing.lateness_s * 1000.0 if pacing else max(0.0, (elapsed_s - deadline_s) * 1000.0)
        row: dict[str, object] = {
            "step": self._step,
            "simulated_time_s": self._sim_time_s,
            "wall_time_s": elapsed_s,
            "total_ms": total_ms,
            "lateness_ms": lateness_ms,
            "deadline_lateness_ms": lateness_ms,
            "pacing_requested_sleep_ms": pacing.requested_sleep_s * 1000.0 if pacing else 0.0,
            "pacing_actual_sleep_ms": pacing.actual_sleep_s * 1000.0 if pacing else 0.0,
            "pacing_rebased": bool(pacing.rebased) if pacing else False,
            "camera_status": camera_status,
        }
        row.update({f"{stage}_ms": self._durations_ns.get(stage, 0) / 1e6 for stage in STAGES})
        self._rows.append(row)
        elapsed_second = int(elapsed_s)
        if elapsed_second > self._last_counter_second:
            self._append_counter(elapsed_s)
            self._last_counter_second = elapsed_second

    def _append_counter(self, elapsed_s: float) -> None:
        recent = self._rows[self._counter_row_index :]
        interval_s = max(elapsed_s - self._counter_wall_s, 1e-9)
        totals = [float(row["total_ms"]) for row in recent]
        simulated = float(self._rows[-1]["simulated_time_s"]) if self._rows else 0.0
        counter: dict[str, object] = {
            "wall_time_s": elapsed_s,
            "simulated_time_s": simulated,
            "real_time_factor": simulated / elapsed_s if elapsed_s else 0.0,
            "effective_physics_hz": len(recent) / interval_s,
            "effective_control_hz": sum(float(row["guidance_control_ms"]) > 0.0 for row in recent) / interval_s,
            "camera_poll_hz": sum(bool(row["camera_status"]) for row in recent) / interval_s,
            "new_frame_hz": (self._camera_counts["new"] - self._counter_camera_new) / interval_s,
            "loop_p50_ms": _percentile(totals, 0.50),
            "loop_p95_ms": _percentile(totals, 0.95),
            "loop_p99_ms": _percentile(totals, 0.99),
            "missed_deadlines": sum(float(row["lateness_ms"]) > 0.0 for row in self._rows),
            "pacing_rebases": sum(bool(row["pacing_rebased"]) for row in self._rows),
            **{f"camera_{key}": value for key, value in self._camera_counts.items()},
            **{f"godot_{key}": value for key, value in self._godot.items()},
        }
        self._counter_rows.append(counter)
        self._counter_row_index = len(self._rows)
        self._counter_wall_s = elapsed_s
        self._counter_camera_new = self._camera_counts["new"]
        if self._echo:
            print(
                "PERF "
                f"wall={elapsed_s:.1f}s sim={simulated:.1f}s rtf={counter['real_time_factor']:.3f} "
                f"loop_p95={counter['loop_p95_ms']:.2f}ms late={counter['missed_deadlines']} "
                f"frames={self._camera_counts['new']}/{self._camera_counts['duplicate']}/{self._camera_counts['missing']}"
            )

    def finish(self, simulated_time_s: float) -> dict[str, object]:
        """Write all performance artifacts and return the aggregate summary."""
        if self._closed:
            return {}
        self._closed = True
        self.output_dir.mkdir(parents=True, exist_ok=True)
        wall_s = (self._clock_ns() - self._started_ns) / 1e9 if self._running else 0.0
        if self._rows and (not self._counter_rows or self._counter_rows[-1]["wall_time_s"] < wall_s):
            self._append_counter(wall_s)
        stage_totals = {
            stage: sum(float(row[f"{stage}_ms"]) for row in self._rows)
            for stage in STAGES
        }
        active_total = sum(stage_totals.values()) or 1.0
        ranked = sorted(
            ({"stage": stage, "total_ms": total, "active_percent": total / active_total * 100.0} for stage, total in stage_totals.items()),
            key=lambda item: item["total_ms"],
            reverse=True,
        )
        loop_values = [float(row["total_ms"]) for row in self._rows]
        summary = {
            "simulated_time_s": simulated_time_s,
            "wall_time_s": wall_s,
            "real_time_factor": simulated_time_s / wall_s if wall_s else 0.0,
            "steps": len(self._rows),
            "physics_hz": self.physics_hz,
            "camera_hz": self.camera_hz,
            "loop_ms": {"p50": _percentile(loop_values, 0.50), "p95": _percentile(loop_values, 0.95), "p99": _percentile(loop_values, 0.99), "max": max(loop_values, default=0.0)},
            "missed_deadlines": sum(float(row["lateness_ms"]) > 0.0 for row in self._rows),
            "maximum_lateness_ms": max((float(row["lateness_ms"]) for row in self._rows), default=0.0),
            "pacing_rebases": sum(bool(row["pacing_rebased"]) for row in self._rows),
            "camera_frames": dict(self._camera_counts),
            "latest_godot": self._godot,
            "ranked_stages": ranked,
        }
        self._write_csv("performance.csv", self._rows)
        self._write_csv("performance-counters.csv", self._counter_rows)
        (self.output_dir / "performance-trace.json").write_text(json.dumps({"traceEvents": self._trace}, separators=(",", ":")) + "\n")
        (self.output_dir / "performance-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        if self._echo:
            print(f"performance: RTF {summary['real_time_factor']:.3f}; wall {wall_s:.2f}s; simulated {simulated_time_s:.2f}s")
            for item in ranked[:5]:
                print(f"  {item['stage']}: {item['total_ms']:.1f} ms ({item['active_percent']:.1f}%)")
            print(f"performance artifacts: {self.output_dir}")
        return summary

    def _write_csv(self, filename: str, rows: list[dict[str, object]]) -> None:
        """Write heterogeneous rows using their union of fields."""
        if not rows:
            (self.output_dir / filename).write_text("")
            return
        fields = list(rows[0])
        fields.extend(key for row in rows for key in row if key not in fields)
        with (self.output_dir / filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
