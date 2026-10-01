"""Command-line adapter for the TTC diagonal-strike package."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from math import isclose
from pathlib import Path

import cv2
import pybullet as p

from .config import SimulationConfig, StrikeConfig
from .config_loader import load_yaml_config
from ..guidance import FlightPhase, GuidanceInput, StrikeGuidance
from .godot_bridge import GodotBridge
from .performance import PerformanceProfiler
from ..sensing import BarometerReading
from .runner import StrikeSimulation
from ..trajectory import TtcDescentPlanner
from ..ttc import BboxTtcTracker


def self_check() -> None:
    config = StrikeConfig()
    seven_inch = SimulationConfig(drone_profile="seven_inch_trainer")
    assert config.simulation.drone_model.mass_kg == 0.65
    assert config.simulation.drone_model.rotor_positions_m[0] == (0.12, 0.12, 0.02)
    assert seven_inch.drone_model.mass_kg == 1.5
    assert seven_inch.drone_model.rotor_positions_m[0] == (0.12, 0.12, 0.025)
    assert seven_inch.drone_model.max_thrust_per_motor_n == 12.0
    assert isclose(seven_inch.physics_settings.propeller_diameter_m, 0.1778)
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation and observation.ttc_s > 0 and observation.raw_ttc_s > 0 and observation.scale_growth_px_s > 0
    assert observation.raw_ttc_s != observation.ttc_s, "Raw and alpha-beta TTC should be distinct"
    tracker.reset()
    assert tracker.update((0, 0, 20, 20), 1.0) is None, "Reset must discard alpha-beta state"
    reset_observation = tracker.update((0, 0, 30, int(config.commit_box_height_px)), 1.1)
    assert reset_observation and reset_observation.ttc_s > 0
    assert tracker.commit_ready, "A large bbox should arm terminal commit"
    planner = TtcDescentPlanner(config)
    trajectory = planner.command(1.0, config.takeoff_altitude_m)
    assert trajectory.forward_velocity_mps == config.forward_speed_mps
    assert trajectory.vertical_velocity_mps == -config.max_descent_velocity_mps
    assert planner.command(None, config.takeoff_altitude_m).vertical_velocity_mps == -config.ttc_unavailable_descent_velocity_mps
    assert planner.command(None, config.takeoff_altitude_m).altitude_target_m == config.takeoff_altitude_m

    guidance = StrikeGuidance(config)
    fast_reading = BarometerReading(0.0, config.takeoff_max_climb_velocity_mps + 0.1)
    guarded_takeoff = guidance.update(GuidanceInput(0.0, fast_reading, None, None, False, False))
    assert guarded_takeoff.thrust_n <= config.hover_thrust_n
    reading = BarometerReading(config.takeoff_altitude_m, 0.0)
    guidance.update(GuidanceInput(0.0, reading, observation, observation, True, False))
    assert guidance.phase == FlightPhase.TRACK
    tracked = guidance.update(GuidanceInput(0.1, reading, observation, observation, True, True))
    assert tracked.pitch_target_rad > 0.0 and tracked.thrust_n > 0.0
    hold = guidance.update(GuidanceInput(0.2, reading, None, observation, True, True))
    assert hold.pitch_target_rad == config.nominal_pitch_rad and hold.trajectory == tracked.trajectory
    committed = guidance.update(GuidanceInput(0.3, reading, None, observation, False, True))
    assert committed.phase == FlightPhase.COMMIT
    assert committed.pitch_target_rad == hold.pitch_target_rad
    settled_commit = guidance.update(GuidanceInput(0.4, BarometerReading(reading.altitude_m, -4.5), None, observation, False, True))
    assert settled_commit.thrust_n > committed.thrust_n, "Commit must preserve the last TTC descent target"
    print("TTC strike component self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--godot", action="store_true", help="Use Godot's FPV camera via Linux shared memory")
    parser.add_argument("--show-plots", action="store_true", help="Show live telemetry plots without PyBullet's GUI")
    parser.add_argument("--interactive", action="store_true", help="Wait for Start and allow Restart from the telemetry window")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=35.0)
    parser.add_argument("--output-root", type=Path, default=Path("outputs/ttc_runs"))
    parser.add_argument("--run-name", type=str)
    parser.add_argument("--config", type=Path, help="Grouped YAML file with simulation and runtime settings")
    parser.add_argument("--video", type=Path)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument("--profile-performance", action="store_true", help="Write detailed loop timings and renderer counters")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check or args.godot else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            try:
                config = load_yaml_config(args.config) if args.config else StrikeConfig()
            except (OSError, ValueError) as exc:
                parser.error(str(exc))
            run_name = args.run_name or datetime.now().strftime("run-%Y%m%d-%H%M%S-%f")
            run_dir = args.output_root / run_name
            run_dir.mkdir(parents=True, exist_ok=False)
            settings = {"simulation": asdict(config.simulation), "runtime": asdict(config.runtime)}
            if args.config:
                settings["source_config"] = str(args.config)
            (run_dir / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
            video = args.video or run_dir / "environment.mp4"
            plot = args.plot or run_dir / "telemetry.png"
            csv = args.csv or run_dir / "telemetry.csv"
            summary = run_dir / "summary.json"
            bridge = GodotBridge() if args.godot else None
            profiler = PerformanceProfiler(
                run_dir,
                config.simulation.physics_settings.physics_hz,
                config.camera_hz,
            ) if args.profile_performance else None
            scenario_name = args.config.stem if args.config else "default"
            result = StrikeSimulation(config, godot=bridge, scenario_name=scenario_name).run(
                not args.headless and not args.godot,
                args.max_seconds,
                None if args.no_video else video,
                None if args.no_plot else plot,
                None if args.no_csv else csv,
                summary,
                args.show_plots,
                args.interactive,
                profiler,
            )
            print(f"run folder: {run_dir}")
            if args.headless and not args.interactive:
                assert result.success, f"Strike failed; impact speed was {result.impact_speed_mps:.1f} m/s"
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)
