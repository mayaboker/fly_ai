"""Rebuild barometer telemetry from a completed TTC run without PyBullet."""

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path

from .config import RuntimeConfig, SimulationConfig, StrikeConfig
from .sensing import Barometer


@dataclass(frozen=True)
class BarometerReplay:
    """Altitude series reconstructed from saved physics telemetry."""

    time_s: list[float]
    true_altitude_m: list[float]
    raw_altitude_m: list[float]
    filtered_altitude_m: list[float]
    phase: list[str]
    collision_time_s: float | None


def _run_paths(path: Path) -> tuple[Path, Path, Path]:
    """Resolve a run folder or CSV path to telemetry, settings, and output."""
    csv_path = path / "telemetry.csv" if path.is_dir() else path
    if csv_path.name != "telemetry.csv":
        raise ValueError("pass a run directory or its telemetry.csv file")
    return csv_path, csv_path.parent / "settings.json", csv_path.parent / "telemetry_barometer.png"


def _load_config(settings_path: Path) -> StrikeConfig:
    """Recreate the saved typed config needed by the deterministic barometer."""
    settings = json.loads(settings_path.read_text())
    return StrikeConfig(
        simulation=SimulationConfig(**settings["simulation"]),
        runtime=RuntimeConfig(**settings["runtime"]),
    )


def replay(csv_path: Path, config: StrikeConfig) -> BarometerReplay:
    """Replay barometer noise over recorded true altitude without simulating physics."""
    rows = list(csv.DictReader(csv_path.open()))
    if not rows:
        raise ValueError(f"{csv_path} contains no telemetry rows")
    barometer = Barometer(config)
    last_reading = None
    raw_altitude_m: list[float] = []
    filtered_altitude_m: list[float] = []
    time_s: list[float] = []
    true_altitude_m: list[float] = []
    phase: list[str] = []
    for row in rows:
        now_s, true_altitude = float(row["time_s"]), float(row["z_m"])
        reading = barometer.sample(true_altitude, now_s)
        if reading is not None:
            last_reading = reading
        if last_reading is None:
            raise AssertionError("the first telemetry row must produce a barometer reading")
        time_s.append(now_s)
        true_altitude_m.append(true_altitude)
        raw_altitude_m.append(last_reading.raw_altitude_m if last_reading.raw_altitude_m is not None else float("nan"))
        filtered_altitude_m.append(last_reading.altitude_m)
        phase.append(row["phase"])
    return BarometerReplay(time_s, true_altitude_m, raw_altitude_m, filtered_altitude_m, phase, _collision_time(csv_path.parent))


def _collision_time(run_directory: Path) -> float | None:
    """Read the recorded collision timestamp when the summary is available."""
    summary_path = run_directory / "summary.json"
    if not summary_path.exists():
        return None
    collision = json.loads(summary_path.read_text()).get("collision")
    return collision.get("time_s") if collision else None


def save_plot(replay_data: BarometerReplay, output: Path) -> None:
    """Save one raw-versus-filtered altitude chart from reconstructed telemetry."""
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(10, 4.5))
    axis.plot(replay_data.time_s, replay_data.raw_altitude_m, "--", color="#f97316", alpha=0.75, label="raw barometer altitude")
    axis.plot(replay_data.time_s, replay_data.filtered_altitude_m, color="#2563eb", linewidth=2, label="filtered barometer altitude")
    axis.plot(replay_data.time_s, replay_data.true_altitude_m, ":", color="#16a34a", label="true PyBullet altitude")
    if replay_data.collision_time_s is not None:
        axis.axvline(replay_data.collision_time_s, color="#b45309", linestyle="--", linewidth=1.2, label="collision")
    tracking = [index for index, phase in enumerate(replay_data.phase) if phase == "track"]
    if tracking:
        axis.axvspan(replay_data.time_s[tracking[0]], replay_data.time_s[tracking[-1]], color="#bfdbfe", alpha=0.28, zorder=0)
    axis.set(xlabel="time (s)", ylabel="altitude (m)", title="Offline BMP388 altitude EMA replay")
    axis.grid(alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=140)
    plt.close(figure)


def main() -> None:
    """Replay one saved run's barometer and save an analysis-only PNG."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Run directory or telemetry.csv path")
    parser.add_argument("--output", type=Path, help="PNG path; defaults beside telemetry.csv")
    args = parser.parse_args()
    csv_path, settings_path, default_output = _run_paths(args.run)
    if not settings_path.exists():
        parser.error(f"missing {settings_path}; cannot reproduce the seeded barometer")
    replay_data = replay(csv_path, _load_config(settings_path))
    output = args.output or default_output
    save_plot(replay_data, output)
    print(f"offline barometer plot: {output}")


if __name__ == "__main__":
    main()
