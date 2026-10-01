"""Nonblocking Matplotlib worker backed by incremental shared telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from math import nan
import multiprocessing as mp
from multiprocessing import shared_memory
from pathlib import Path
from queue import Empty, Full
from typing import Final

import numpy as np

from .config import SceneConfig, StrikeConfig
from .telemetry import FlightLog, make_plot, move_plot_window, refresh_plot


LIVE_FIELDS: Final = (
    "time_s", "x_m", "z_m", "vx_mps", "vz_mps",
    "command_vx_mps", "command_vz_mps", "pid_vz_target_mps",
    "command_altitude_m", "command_thrust_n", "command_pitch_deg",
    "measured_pitch_deg", "raw_bbox_growth_px_s", "bbox_growth_px_s",
    "barometer_raw_altitude_m", "barometer_filtered_altitude_m",
)
PHASES: Final = ("takeoff", "track", "commit", "abort", "post-impact")


@dataclass(frozen=True)
class LivePlotSpec:
    """Picklable description of the shared plot buffer."""

    shared_name: str
    capacity: int
    columns: int
    refresh_hz: float


def _offer_command(queue, command: str) -> None:
    """Send a rare UI command without ever blocking the plot process."""
    try:
        queue.put_nowait(command)
    except Full:
        pass


def _append_rows(log: FlightLog, rows: np.ndarray) -> None:
    """Append newly published matrix rows to a worker-local flight log."""
    for index, field in enumerate(LIVE_FIELDS):
        getattr(log, field).extend(rows[:, index].tolist())
    log.phase.extend(PHASES[min(max(int(code), 0), len(PHASES) - 1)] for code in rows[:, -1])


def _plot_worker(spec: LivePlotSpec, state, collision_time, stop_event, commands, scene: SceneConfig, scenario_name: str, position_px: tuple[int, int], controls: bool) -> None:
    """Own the GUI event loop and incrementally consume immutable shared rows."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    block = shared_memory.SharedMemory(name=spec.shared_name)
    matrix = np.ndarray((spec.capacity, spec.columns), dtype=np.float64, buffer=block.buf)
    plot = make_plot(StrikeConfig(), scene, scenario_name)
    plot.figure.canvas.manager.set_window_title(f"TTC strike telemetry — {scene.drone_profile} / {scenario_name}")
    move_plot_window(plot, position_px)
    widgets = []
    if controls:
        for left, label, command in ((0.25, "Start", "start"), (0.39, "Pause", "pause"), (0.53, "Restart", "restart"), (0.67, "Stop", "stop")):
            button = Button(plot.figure.add_axes((left, 0.01, 0.11, 0.025)), label)
            button.on_clicked(lambda _event, value=command: _offer_command(commands, value))
            widgets.append(button)
        plot.figure.subplots_adjust(bottom=0.05)
        plot.figure.canvas.mpl_connect("close_event", lambda _event: _offer_command(commands, "stop"))
    plt.show(block=False)
    log = FlightLog()
    generation = -1
    consumed = 0
    refresh_period_s = 1.0 / spec.refresh_hz
    next_refresh = 0.0
    try:
        while not stop_event.is_set() and plt.fignum_exists(plot.figure.number):
            with state.get_lock():
                published_generation, count = int(state[0]), int(state[1])
            if published_generation != generation:
                generation = published_generation
                consumed = 0
                log = FlightLog()
            now = __import__("time").monotonic()
            if now >= next_refresh and count > consumed:
                rows = matrix[consumed:count].copy()
                _append_rows(log, rows)
                consumed = count
                with collision_time.get_lock():
                    log.collision_time_s = None if np.isnan(collision_time.value) else collision_time.value
                refresh_plot(plot, log)
                next_refresh = now + refresh_period_s
            plt.pause(0.02)
    finally:
        block.close()
        plt.close(plot.figure)


class LivePlotProcess:
    """Publish live chart samples to a spawned, independently rendered process."""

    def __init__(self, config: StrikeConfig, scene: SceneConfig, scenario_name: str, max_seconds: float, controls: bool = False, refresh_hz: float = 2.0) -> None:
        context = mp.get_context("spawn")
        capacity = max(1, round(max_seconds * scene.physics_settings.physics_hz) + 1)
        columns = len(LIVE_FIELDS) + 1
        size = capacity * columns * np.dtype(np.float64).itemsize
        self._block = shared_memory.SharedMemory(create=True, size=size)
        self._matrix = np.ndarray((capacity, columns), dtype=np.float64, buffer=self._block.buf)
        self._spec = LivePlotSpec(self._block.name, capacity, columns, refresh_hz)
        self._state = context.Array("Q", (0, 0), lock=True)
        self._collision_time = context.Value("d", nan, lock=True)
        self._stop_event = context.Event()
        self._commands = context.Queue(maxsize=8)
        self._process = context.Process(
            target=_plot_worker,
            args=(self._spec, self._state, self._collision_time, self._stop_event, self._commands, scene, scenario_name, config.plot_window_position_px, controls),
            name="fly-smart-live-plot",
            daemon=True,
        )
        self._process.start()
        self._closed = False
        self._capacity_warned = False
        self._worker_warned = False

    def publish(self, log: FlightLog) -> None:
        """Append the newest complete log row without waiting for Matplotlib."""
        if not self._process.is_alive():
            if not self._worker_warned:
                print("Live plot process stopped; offline telemetry recording continues")
                self._worker_warned = True
            return
        with self._state.get_lock():
            row = int(self._state[1])
        if row >= self._spec.capacity:
            if not self._capacity_warned:
                print("Live plot shared buffer is full; offline telemetry recording continues")
                self._capacity_warned = True
            return
        for column, field in enumerate(LIVE_FIELDS):
            self._matrix[row, column] = getattr(log, field)[-1]
        phase = log.phase[-1]
        self._matrix[row, -1] = PHASES.index(phase) if phase in PHASES else len(PHASES) - 1
        with self._state.get_lock():
            self._state[1] = row + 1

    def mark_collision(self, time_s: float) -> None:
        """Publish the collision marker independently of sample rows."""
        with self._collision_time.get_lock():
            self._collision_time.value = time_s

    def reset(self) -> None:
        """Start a new plot generation using the existing allocation."""
        with self._state.get_lock():
            self._state[0] += 1
            self._state[1] = 0
        with self._collision_time.get_lock():
            self._collision_time.value = nan
        self._capacity_warned = False

    def read_commands(self) -> list[str]:
        """Drain low-rate control commands emitted by plot buttons."""
        values = []
        while True:
            try:
                values.append(self._commands.get_nowait())
            except Empty:
                return values

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    def close(self) -> None:
        """Stop the worker and release the owned shared-memory segment."""
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._commands.close()
        self._block.close()
        self._block.unlink()
