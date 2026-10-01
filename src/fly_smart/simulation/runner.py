"""PyBullet adapter that composes sensing, TTC, guidance, views, and telemetry."""

from dataclasses import dataclass
from contextlib import nullcontext
from math import degrees, sqrt
from pathlib import Path
import time

import cv2
import pybullet as p

from .drone_physics import PhysicsEngine, clamp
from ..common.flight_control import AttitudeController
from .pybullet_sensors import read_imu
from .pybullet_utils import create_world, draw_force_vectors, reset_drone
from .gui_helper import SimulationControls
from .forward_camera import add_environment_buildings, add_red_cube, forward_rgb
from .godot_bridge import GodotBridge
from .live_plot import LivePlotProcess
from .performance import PerformanceProfiler
from .pacing import PacingResult, RealTimePacer
from ..red_target_detector import detect_red_box

from .config import SceneConfig, StrikeConfig
from ..guidance import FlightPhase, GuidanceCommand, GuidanceInput, StrikeGuidance
from ..sensing import BarometerReading, VerticalEstimator
from .sensors import Barometer, VerticalImu
from .telemetry import FlightLog, build_summary, save_csv, save_plot, save_summary
from ..ttc import BboxTtcTracker, TtcObservation
from .views import annotate, environment_rgb

@dataclass(frozen=True)
class StrikeResult:
    success: bool
    phase: str
    simulated_time_s: float
    impact_speed_mps: float
    video: Path | None
    plot: Path | None
    csv: Path | None
    summary: Path | None


class StrikeSimulation:
    """Run one configured strike against the concrete PyBullet simulator."""

    def __init__(self, config: StrikeConfig | None = None, scene: SceneConfig | None = None, godot: GodotBridge | None = None, scenario_name: str = "default") -> None:
        self.config = config or StrikeConfig()
        self.scene = scene or self.config.simulation
        self.godot = godot
        self.scenario_name = scenario_name

    def run(self, gui: bool, max_seconds: float, video: Path | None, plot: Path | None, csv: Path | None = None, summary: Path | None = None, show_plots: bool = False, interactive: bool = False, profiler: PerformanceProfiler | None = None) -> StrikeResult:
        config = self.config
        model = self.scene.drone_model
        settings = self.scene.physics_settings
        physics_hz = settings.physics_hz
        time_step = settings.time_step_s
        control_steps = settings.control_steps
        drone = create_world(model, settings)
        engine = PhysicsEngine(model, settings)
        p.resetBasePositionAndOrientation(drone, config.launch_position, (0, 0, 0, 1))
        cube = add_red_cube(self.scene.target_center, self.scene.target_size_m, collision=self.godot is None)
        add_environment_buildings()
        if self.godot:
            self.godot.open()
        barometer, tracker, guidance = Barometer(config), BboxTtcTracker(config), StrikeGuidance(config)
        vertical_imu = VerticalImu(config)
        vertical_estimator = VerticalEstimator(config, config.launch_position[2])
        previous_vertical_velocity_mps = 0.0
        attitude_controller = AttitudeController(config.pitch_attitude_pid_gains)
        torque = (0.0, 0.0, 0.0)
        command = GuidanceCommand(FlightPhase.TAKEOFF, config.hover_thrust_n, 0.0, None)
        baro = BarometerReading(config.launch_position[2], 0.0)
        observation: TtcObservation | None = None
        target_visible = False
        force_lines = [-1, -1, -1, -1]
        renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER
        impact_speed, stop_at_s = 0.0, None
        log = FlightLog()
        writer = None
        attempt_number = 0
        attempt_video, attempt_plot, attempt_csv, attempt_summary = video, plot, csv, summary
        live_plot = self._live_plot(gui or show_plots or interactive, plot, config, self.scene, max_seconds, interactive and self.godot is None)
        controls: SimulationControls | None = SimulationControls() if interactive and (self.godot or live_plot) else None
        show_frame = gui
        latest_profile_time_s = 0.0
        attempt_wall_start: float | None = None
        pacer = RealTimePacer(time_step) if gui or self.godot else None

        def measure(stage: str):
            """Return an active profiler span or a zero-cost null context."""
            return profiler.measure(stage) if profiler else nullcontext()

        def apply_control_commands() -> None:
            """Apply commands from Godot or the compatibility plot controls."""
            if controls is None:
                return
            commands = live_plot.read_commands() if live_plot else []
            if self.godot:
                commands.extend(self.godot.read_control_events())
            before = (controls.running, controls.reset_requested, controls.exit_requested)
            for value in commands:
                if value == "start":
                    controls.start()
                elif value == "pause":
                    controls.stop()
                elif value == "restart":
                    controls.request_reset()
                elif value == "stop":
                    controls.stop()
                    controls.request_exit()
            if self.godot and before != (controls.running, controls.reset_requested, controls.exit_requested):
                self.godot.publish_control_state(True, controls.running)
        if show_frame:
            cv2.namedWindow("TTC diagonal strike", cv2.WINDOW_NORMAL)
            cv2.moveWindow("TTC diagonal strike", *config.opencv_window_position_px)
        if gui:
            p.resetDebugVisualizerCamera(36.0, 48.0, -25.0, (7.0, 0.0, 7.0))

        def reset_attempt() -> None:
            """Restore the complete flight state and publish Godot's initial pose."""
            nonlocal attempt_number, attempt_video, attempt_plot, attempt_csv, attempt_summary
            nonlocal engine, barometer, tracker, guidance, vertical_imu, vertical_estimator
            nonlocal previous_vertical_velocity_mps, attitude_controller, torque, command, baro
            nonlocal observation, target_visible, impact_speed, stop_at_s, log, writer
            nonlocal attempt_wall_start
            if writer:
                writer.release()
            if interactive and attempt_number:
                if attempt_plot:
                    save_plot(log, config, self.scene, attempt_plot, self.scenario_name)
                if attempt_csv:
                    save_csv(log, attempt_csv)
                if attempt_summary:
                    save_summary(
                        build_summary(
                            log,
                            config,
                            self.scene,
                            False,
                            command.phase.value,
                            log.time_s[-1] if log.time_s else 0.0,
                            {"video": attempt_video, "plot": attempt_plot, "csv": attempt_csv, "summary": attempt_summary},
                            "Restarted",
                        ),
                        attempt_summary,
                    )
            attempt_number += 1
            if interactive and summary:
                attempt_dir = summary.parent / f"attempt-{attempt_number:03d}"
                attempt_video = attempt_dir / video.name if video else None
                attempt_plot = attempt_dir / plot.name if plot else None
                attempt_csv = attempt_dir / csv.name if csv else None
                attempt_summary = attempt_dir / summary.name
            writer = self._video_writer(attempt_video, config)
            reset_drone(drone, config.launch_position)
            p.resetBasePositionAndOrientation(cube, self.scene.target_center, (0, 0, 0, 1))
            p.resetBaseVelocity(cube, (0, 0, 0), (0, 0, 0))
            engine = PhysicsEngine(model, settings)
            barometer, tracker, guidance = Barometer(config), BboxTtcTracker(config), StrikeGuidance(config)
            vertical_imu = VerticalImu(config)
            vertical_estimator = VerticalEstimator(config, config.launch_position[2])
            previous_vertical_velocity_mps = 0.0
            attitude_controller = AttitudeController(config.pitch_attitude_pid_gains)
            torque = (0.0, 0.0, 0.0)
            command = GuidanceCommand(FlightPhase.TAKEOFF, config.hover_thrust_n, 0.0, None)
            baro = BarometerReading(config.launch_position[2], 0.0)
            observation, target_visible = None, False
            impact_speed, stop_at_s = 0.0, None
            attempt_wall_start = None
            if pacer:
                pacer.reset()
            log = FlightLog()
            if live_plot:
                live_plot.reset()
            if self.godot:
                self.godot.clear_collision_events()
                target_position, target_orientation = p.getBasePositionAndOrientation(cube)
                self.godot.publish_pose(config.launch_position, (0, 0, 0, 1), target_position, target_orientation, reset=True)
                self.godot.publish_control_state(interactive, controls.running if controls else True)

        reset_attempt()

        def finish(success: bool, phase: str, now_s: float, abort_reason: str | None = None) -> StrikeResult:
            nonlocal latest_profile_time_s
            latest_profile_time_s = now_s
            if profiler:
                profiler.finish(now_s)
            if attempt_plot:
                save_plot(log, config, self.scene, attempt_plot, self.scenario_name)
            if attempt_csv:
                save_csv(log, attempt_csv)
            result = StrikeResult(success, phase, now_s, impact_speed, attempt_video, attempt_plot, attempt_csv, attempt_summary)
            summary_data = build_summary(log, config, self.scene, success, phase, now_s, {"video": attempt_video, "plot": attempt_plot, "csv": attempt_csv, "summary": attempt_summary}, abort_reason)
            if attempt_summary:
                save_summary(summary_data, attempt_summary)
            self._print_summary(result, summary_data)
            return result

        def finish_if_disconnected(now_s: float) -> StrikeResult | None:
            """Save the partial run when closing the PyBullet GUI disconnects its server."""
            if p.isConnected():
                return None
            return finish(False, command.phase.value, now_s, "PyBullet physics server closed")

        try:
            step = 0
            while step < round(max_seconds / time_step):
                if controls:
                    apply_control_commands()
                    while not controls.running:
                        if pacer:
                            pacer.reset()
                        if controls.exit_requested:
                            return finish(False, command.phase.value, step * time_step, "Interactive session closed")
                        if controls.consume_reset():
                            reset_attempt()
                            step = 0
                        apply_control_commands()
                        time.sleep(0.02)
                        if show_frame:
                            cv2.waitKey(1)
                    if controls.consume_reset():
                        reset_attempt()
                        step = 0
                        continue
                now_s = step * time_step
                if attempt_wall_start is None:
                    attempt_wall_start = time.perf_counter()
                if pacer and not pacer.active:
                    pacer.start()
                latest_profile_time_s = now_s
                if profiler:
                    profiler.begin_step(step, now_s)
                disconnected = finish_if_disconnected(now_s)
                if disconnected:
                    return disconnected
                with measure("state_sensors"):
                    position, _ = p.getBasePositionAndOrientation(drone)
                    current_vertical_velocity_mps = p.getBaseVelocity(drone)[0][2]
                    imu = vertical_imu.sample((current_vertical_velocity_mps - previous_vertical_velocity_mps) / time_step, now_s)
                    previous_vertical_velocity_mps = current_vertical_velocity_mps
                    sample = barometer.sample(position[2], now_s)
                    estimate = vertical_estimator.update(imu, time_step, sample)
                    baro = BarometerReading(estimate.altitude_m, estimate.vertical_velocity_mps, sample.raw_altitude_m if sample else None)

                frame = None
                box = None
                camera_status = ""
                camera_tick = step % (physics_hz // config.camera_hz) == 0
                if camera_tick:
                    if self.godot:
                        drone_position, drone_orientation = p.getBasePositionAndOrientation(drone)
                        target_position, target_orientation = p.getBasePositionAndOrientation(cube)
                        with measure("camera_pose"):
                            self.godot.publish_pose(drone_position, drone_orientation, target_position, target_orientation)
                        with measure("frame_copy"):
                            frame_sample = self.godot.read_frame_sample()
                        if profiler:
                            camera_status = profiler.camera_frame(frame_sample.sequence if frame_sample else None)
                        if frame_sample is not None:
                            with measure("detection"):
                                frame, box = detect_red_box(frame_sample.image)
                            if writer:
                                with measure("video_encode"):
                                    writer.write(cv2.resize(frame, config.environment_size_px))
                            target_visible = box is not None
                            observation = tracker.update(box, now_s)
                    else:
                        if writer:
                            with measure("video_encode"):
                                writer.write(cv2.cvtColor(environment_rgb(renderer, config), cv2.COLOR_RGB2BGR))
                        with measure("frame_copy"):
                            camera_image = forward_rgb(
                                drone,
                                renderer,
                                look_down_degrees=config.camera_look_down_deg,
                                width_px=config.camera_width_px,
                                height_px=config.camera_height_px,
                                fov_deg=config.camera_fov_deg,
                            )
                        with measure("detection"):
                            frame, box = detect_red_box(camera_image)
                        target_visible = box is not None
                        observation = tracker.update(box, now_s)

                if step % control_steps == 0:
                    with measure("guidance_control"):
                        command, observation, torque = self._update_control(
                            stop_at_s, guidance, now_s, baro, observation, tracker,
                            target_visible, drone, attitude_controller, command, torque,
                        )
                    if command.commit_expired:
                        print("Commit deadline expired without contact")
                        return finish(False, command.phase.value, now_s)
                    if command.phase == FlightPhase.ABORT:
                        last_height = tracker.last_observation.box[3] if tracker.last_observation else 0
                        return finish(False, command.phase.value, now_s, f"target lost before commit (last bbox height {last_height:g} px)")

                # thrust_n is the collective force. Split it evenly before
                # mapping force to a PWM signal for the four motors.
                collective = 0.0 if stop_at_s is not None else command.thrust_n
                pwm = engine.pwm_from_thrust(clamp(collective / 4, 0.0, model.max_thrust_per_motor_n))
                incoming_velocity = p.getBaseVelocity(drone)[0]
                with measure("physics"):
                    flight_step = engine.step(drone, pwm, torque)
                disconnected = finish_if_disconnected(now_s)
                if disconnected:
                    return disconnected
                position, _ = p.getBasePositionAndOrientation(drone)
                velocity, _ = p.getBaseVelocity(drone)
                pitch_rad = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1]
                pitch_torque = torque[1]
                if stop_at_s is None:
                    # Record the collision sample, then freeze telemetry while
                    # passive post-impact physics continues for the video.
                    log.append(now_s, position, velocity, command, pitch_rad, pitch_torque, observation, flight_step, baro)
                    if live_plot:
                        with measure("plot_display"):
                            live_plot.publish(log)

                if self.godot and camera_tick:
                    wall_elapsed_s = time.perf_counter() - attempt_wall_start
                    completed_simulated_time_s = (step + 1) * time_step
                    trajectory = command.trajectory
                    with measure("telemetry_publish"):
                        self.godot.publish_telemetry({
                        "time_s": now_s,
                        "wall_elapsed_s": wall_elapsed_s,
                        "real_time_factor": completed_simulated_time_s / wall_elapsed_s if wall_elapsed_s > 0.0 else None,
                        "phase": command.phase.value,
                        "position_m": position,
                        "velocity_mps": velocity,
                        "altitude_m": baro.altitude_m,
                        "vertical_velocity_mps": baro.vertical_velocity_mps,
                        "measured_pitch_deg": degrees(pitch_rad),
                        "commanded_pitch_deg": degrees(command.pitch_target_rad),
                        "thrust_n": command.thrust_n,
                        "target_visible": target_visible,
                        "bbox": box,
                        "scale_px": observation.scale_px if observation else None,
                        "growth_px_s": observation.scale_growth_px_s if observation else None,
                        "ttc_s": observation.ttc_s if observation else None,
                        "command_vx_mps": trajectory.forward_velocity_mps if trajectory else None,
                        "command_vz_mps": trajectory.vertical_velocity_mps if trajectory else None,
                        })

                with measure("collision"):
                    collision_kind = self.godot.read_collision_event() if self.godot else ("target" if p.getContactPoints(drone, cube) else None)
                    if profiler and self.godot:
                        profiler.update_godot(self.godot.read_performance_metrics())
                if stop_at_s is None and collision_kind == "target":
                    impact_speed = sqrt(sum(component**2 for component in incoming_velocity))
                    log.mark_collision(now_s, position, incoming_velocity)
                    stop_at_s = now_s + config.post_impact_seconds
                    print(f"Impact: {impact_speed:.1f} m/s; recording aftermath for {config.post_impact_seconds:.0f} s")
                    if live_plot:
                        live_plot.mark_collision(now_s)
                elif stop_at_s is None and collision_kind == "obstacle":
                    return finish(False, command.phase.value, now_s, "Godot obstacle collision")
                if stop_at_s is not None and now_s >= stop_at_s:
                    # Contact is the geometry-free success condition.  Keep
                    # impact speed as telemetry instead of rejecting a valid
                    # strike because the simulated vehicle model is tuned
                    # differently from a real airframe.
                    return finish(True, "post-impact", now_s)

                with measure("plot_display"):
                    if show_frame:
                        if frame is not None:
                            cv2.imshow("TTC diagonal strike", annotate(frame, command, observation))
                            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                                return finish(False, command.phase.value, now_s)
                    if gui:
                        draw_force_vectors(drone, flight_step, force_lines)
                if controls and controls.consume_reset():
                    reset_attempt()
                    step = 0
                    continue
                pacing_result: PacingResult | None = None
                if pacer:
                    with measure("sleep"):
                        pacing_result = pacer.wait()
                if profiler:
                    profiler.end_step(camera_status, pacing_result)
                step += 1
            print(f"Strike timed out in {command.phase.value} phase")
            return finish(False, command.phase.value, max_seconds)
        except p.error:
            # The GUI can close between two PyBullet calls (for example while
            # the forward camera renders).  Preserve telemetry in that case;
            # other PyBullet failures remain visible to the caller.
            if not p.isConnected():
                return finish(False, command.phase.value, now_s, "PyBullet physics server closed")
            raise
        finally:
            if profiler:
                profiler.finish(latest_profile_time_s)
            if writer:
                writer.release()
            if live_plot:
                live_plot.close()
            if self.godot:
                self.godot.publish_control_state(False, False)
                self.godot.close()

    @staticmethod
    def _update_control(
        stop_at_s: float | None,
        guidance: StrikeGuidance,
        now_s: float,
        baro: BarometerReading,
        observation: TtcObservation | None,
        tracker: BboxTtcTracker,
        target_visible: bool,
        drone: int,
        attitude_controller: AttitudeController,
        command: GuidanceCommand,
        torque: tuple[float, float, float],
    ) -> tuple[GuidanceCommand, TtcObservation | None, tuple[float, float, float]]:
        """Run one control update while keeping its profiler boundary explicit."""
        if stop_at_s is not None:
            return command, observation, (0.0, 0.0, 0.0)
        current_velocity = p.getBaseVelocity(drone)[0]
        measured_pitch = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1]
        command = guidance.update(GuidanceInput(
            now_s,
            baro,
            observation,
            tracker.last_observation,
            target_visible,
            tracker.commit_ready,
            current_velocity[0],
            measured_pitch,
        ))
        if command.reset_ttc:
            tracker.reset()
            observation = None
        if not command.commit_expired and command.phase != FlightPhase.ABORT:
            torque = attitude_controller.update(read_imu(drone), yaw_target=0.0, pitch_target=command.pitch_target_rad)
        return command, observation, torque

    @staticmethod
    def _video_writer(video: Path | None, config: StrikeConfig):
        if not video:
            return None
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), config.camera_hz, config.environment_size_px)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video output: {video}")
        return writer

    def _live_plot(self, gui: bool, output: Path | None, config: StrikeConfig, scene: SceneConfig, max_seconds: float, controls: bool) -> LivePlotProcess | None:
        """Create the optional live telemetry window for this named scenario."""
        if not gui or not output:
            return None
        return LivePlotProcess(config, scene, self.scenario_name, max_seconds, controls=controls)

    def _print_summary(self, result: StrikeResult, summary: dict[str, object]) -> None:
        """Print one colored, human-readable summary for the completed run."""
        reset, cyan, green, red, yellow = "\033[0m", "\033[1;36m", "\033[1;32m", "\033[1;31m", "\033[1;33m"
        status = f"{green}target contacted{reset}" if result.success else f"{red}no valid contact{reset}"
        print(f"\n{cyan}--- TTC strike summary ---{reset}")
        print(f"drone: {yellow}{self.scene.drone_profile}{reset}; scenario: {yellow}{self.scenario_name}{reset}")
        print(f"result: {status}")
        print(f"final phase: {result.phase}; simulated time: {result.simulated_time_s:.1f} s")
        if result.phase == FlightPhase.ABORT.value and summary.get("abort_reason"):
            print(f"{red}ABORT: {summary['abort_reason']}{reset}")
        collision = summary["collision"]
        metrics = summary["flight_metrics"]
        print(f"starting pose: {summary['starting_pose']['position_m']}")
        print(f"target: center {summary['target']['center_m']}, size {summary['target']['size_m']:.2f} m")
        print(f"collision time: {collision['time_s']}")
        print(f"collision position: {collision['position_m']}")
        print(f"hitting velocity: {collision['velocity_mps']}")
        print(f"maximum altitude: {metrics['maximum_altitude_m']}")
        print(f"maximum forward speed: {metrics['maximum_forward_speed_mps']}")
        if result.impact_speed_mps:
            print(f"impact speed: {result.impact_speed_mps:.1f} m/s")
        if result.video:
            print(f"environment video: {result.video}")
        if result.plot:
            print(f"trajectory plot: {result.plot}")
        if result.csv:
            print(f"telemetry CSV: {result.csv}")
        if result.summary:
            print(f"run summary: {result.summary}")
        environment = "Terrain3D forest, road, rocks, trees, and red target cube" if self.godot else "red target cube and 3 static buildings"
        print(f"environment: {environment}")
