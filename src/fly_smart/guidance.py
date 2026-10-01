"""Flight phases and high-level pitch/thrust guidance."""

from dataclasses import dataclass, replace
from enum import Enum
from math import cos, radians

from .common.pid import PID

from .mission import MissionConfig
from .sensing import BarometerReading
from .trajectory import TrajectoryCommand, TtcDescentPlanner
from .ttc import TtcObservation


TTC_PITCH_RELEASE_FRACTION = 0.10
TTC_PITCH_RELEASE_SAMPLES = 3
PITCH_TARGET_SLEW_RAD_S = radians(20.0)
VERTICAL_TARGET_SLEW_MPS2 = 2.0


class FlightPhase(str, Enum):
    TAKEOFF = "takeoff"
    TRACK = "track"
    COMMIT = "commit"
    ABORT = "abort"


@dataclass(frozen=True)
class GuidanceInput:
    now_s: float
    barometer: BarometerReading
    observation: TtcObservation | None
    last_observation: TtcObservation | None
    target_visible: bool
    commit_ready: bool
    forward_velocity_mps: float = 0.0
    measured_pitch_rad: float = 0.0


@dataclass(frozen=True)
class GuidanceCommand:
    """High-level command consumed by ``StrikeSimulation``."""

    phase: FlightPhase
    thrust_n: float
    pitch_target_rad: float
    trajectory: TrajectoryCommand | None
    reset_ttc: bool = False
    commit_expired: bool = False
    vertical_velocity_target_mps: float | None = None


class StrikeGuidance:
    """Translate sensor observations into pitch and collective-thrust commands.

    The guidance layer owns three independent PID controllers:

    - ``altitude_pid`` runs during ``TAKEOFF``. It converts altitude error and
      measured vertical velocity into collective thrust; a separate speed
      guard caps thrust at hover above the configured climb limit.
    - ``forward_pid`` runs during ``TRACK``. It converts forward-velocity
      error into a positive pitch target, which tilts the rotor disk to build
      forward speed.
    - ``vertical_velocity_pid`` runs during ``TRACK``. It converts the planned
      vertical-velocity error into collective thrust so the vehicle follows the
      altitude trajectory while moving forward.

    ``ABORT`` reuses ``altitude_pid`` with zero altitude error to damp vertical
    velocity. ``COMMIT`` freezes the last pitch and descent-rate target, then
    continues the barometer-driven vertical PID without camera measurements.
    """

    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.phase = FlightPhase.TAKEOFF
        self.trajectory = TtcDescentPlanner(config)
        self.altitude_pid = PID(*config.altitude_pid_gains, integral_limit=config.altitude_integral_limit)
        self.forward_pid = PID(*config.forward_speed_pid_gains, integral_limit=config.forward_pitch_integral_limit)
        self.vertical_velocity_pid = PID(*config.vertical_velocity_pid_gains)
        self.last_command = GuidanceCommand(self.phase, config.hover_thrust_n, 0.0, None)
        self.commit_deadline_s: float | None = None
        self.commit_descent_velocity_mps: float | None = None
        self.last_tracking_descent_velocity_mps: float | None = None
        self._acquisition_boost_active = False
        self._acquisition_ttc_baseline_s: float | None = None
        self._acquisition_release_samples = 0
        self._last_processed_observation: TtcObservation | None = None
        self._last_valid_tracking_ttc_s: float | None = None
        self._last_track_update_s: float | None = None
        self._last_pitch_target_rad: float | None = None
        self._last_vertical_target_mps: float | None = None

    def update(self, data: GuidanceInput) -> GuidanceCommand:
        """Advance the guidance state machine by one control tick.

        Phase flow:
        - ``TAKEOFF`` uses altitude PID with a bounded measured climb rate and
          zero pitch until altitude and vertical speed settle; the first fresh
          camera observation starts ``TRACK``.
        - ``TRACK`` converts visible-target TTC and barometer readings into a
          forward-velocity and vertical-velocity trajectory command.
        - Losing the target enters ``COMMIT`` only after a valid final TTC
          observation; it otherwise enters ``ABORT``.
        - ``COMMIT`` keeps the final pitch and descent-rate target while its
          vertical PID updates collective thrust until the TTC deadline.
        - ``ABORT`` removes pitch and damps vertical motion with hover thrust.

        Returns a ``GuidanceCommand`` for the current phase. ``reset_ttc``
        tells the simulation to discard observations gathered before tracking,
        and ``commit_expired`` marks the end of the bounded commit interval.
        """
        if self.phase == FlightPhase.TAKEOFF:
            # Hold pitch at zero while the altitude loop climbs to the camera
            # observation height. Above the speed limit, never add more than
            # hover thrust, so gravity and drag brake the vehicle.
            thrust = self.config.hover_thrust_n + self.altitude_pid.update(
                self.config.takeoff_altitude_m - data.barometer.altitude_m,
                data.barometer.vertical_velocity_mps,
            )
            if data.barometer.vertical_velocity_mps >= self.config.takeoff_max_climb_velocity_mps:
                thrust = min(thrust, self.config.hover_thrust_n)
            command = GuidanceCommand(self.phase, thrust, 0.0, None)
            ready = data.barometer.altitude_m > self.config.takeoff_altitude_m - self.config.takeoff_altitude_tolerance_m
            stable = abs(data.barometer.vertical_velocity_mps) < self.config.takeoff_velocity_tolerance_mps
            if ready and stable and data.last_observation:
                self.phase = FlightPhase.TRACK
                self.forward_pid.reset()
                self.last_tracking_descent_velocity_mps = None
                self._acquisition_boost_active = self.config.ttc_unavailable_pitch_boost_rad > 0.0
                self._acquisition_ttc_baseline_s = None
                self._acquisition_release_samples = 0
                self._last_processed_observation = None
                self._last_valid_tracking_ttc_s = None
                self._last_track_update_s = None
                self._last_pitch_target_rad = None
                self._last_vertical_target_mps = None
                # The camera estimate was accumulated during takeoff. Ignore
                # it for this first track command and reset the tracker, so a
                # fresh observation starts the tracking phase.
                command = replace(self._track_command(replace(data, observation=None)), reset_ttc=True)
            self.last_command = command
            return command

        if self.phase == FlightPhase.TRACK and not data.target_visible:
            # TRACK uses live camera observations to update the trajectory. If
            # the target leaves view after the configured commit condition,
            # freeze the final pitch and corrected descent-rate target for the
            # short predicted remaining time. If there is no reliable final
            # observation, stop tracking instead.
            if data.commit_ready and data.last_observation:
                self.phase = FlightPhase.COMMIT
                self.commit_deadline_s = data.now_s + data.last_observation.ttc_s + self.config.commit_timeout_margin_s
                # A control tick can run after the last camera frame. Preserve
                # the descent target from the last *valid TTC* observation,
                # not a later no-observation altitude-hold command.
                self.commit_descent_velocity_mps = self.last_tracking_descent_velocity_mps
            else:
                self.phase = FlightPhase.ABORT

        if self.phase == FlightPhase.COMMIT:
            # Keep lateral guidance blind and fixed, but keep altitude control
            # closed-loop: constant thrust would eventually arrest descent or
            # cause a climb as the airframe attitude and velocity change.
            desired_vz = self.commit_descent_velocity_mps or 0.0
            vertical_force = self.config.hover_thrust_n + self.vertical_velocity_pid.update(
                desired_vz - data.barometer.vertical_velocity_mps,
                0.0,
            )
            thrust = vertical_force / max(cos(data.measured_pitch_rad), 0.5)
            command = GuidanceCommand(
                self.phase,
                thrust,
                self.last_command.pitch_target_rad,
                self.last_command.trajectory,
                commit_expired=data.now_s > (self.commit_deadline_s or data.now_s),
                vertical_velocity_target_mps=desired_vz,
            )
            self.last_command = command
            return command

        if self.phase == FlightPhase.ABORT:
            # ABORT removes the forward-pitch command and asks the altitude
            # loop to damp vertical motion around the current altitude.
            thrust = self.config.hover_thrust_n + self.altitude_pid.update(0.0, data.barometer.vertical_velocity_mps)
            return GuidanceCommand(self.phase, thrust, 0.0, None)

        # TRACK has a visible target. Convert its current TTC estimate into a
        # forward/descent trajectory, then let the velocity loops form pitch
        # and collective-thrust commands in _track_command().
        return self._track_command(data)

    def _track_command(self, data: GuidanceInput) -> GuidanceCommand:
        self._update_acquisition_state(data.observation)
        effective_ttc_s = data.observation.ttc_s if data.observation else self._last_valid_tracking_ttc_s
        trajectory = self.trajectory.command(
            effective_ttc_s,
            data.barometer.altitude_m,
        )
        pitch_correction = self.forward_pid.update(
            trajectory.forward_velocity_mps - data.forward_velocity_mps,
            0.0,
        )
        pitch = max(0.0, min(self.config.max_pitch_rad, pitch_correction))
        if data.target_visible and self._acquisition_boost_active:
            pitch = min(self.config.max_pitch_rad + self.config.ttc_unavailable_pitch_boost_rad, pitch + self.config.ttc_unavailable_pitch_boost_rad)
        corrected_vz = self._corrected_vertical_velocity(trajectory, data.barometer.altitude_m)
        dt_s = max(0.0, data.now_s - self._last_track_update_s) if self._last_track_update_s is not None else 0.0
        if dt_s > 0.0 and self._last_pitch_target_rad is not None:
            pitch = self._slew(self._last_pitch_target_rad, pitch, PITCH_TARGET_SLEW_RAD_S * dt_s)
        if dt_s > 0.0 and self._last_vertical_target_mps is not None:
            corrected_vz = self._slew(self._last_vertical_target_mps, corrected_vz, VERTICAL_TARGET_SLEW_MPS2 * dt_s)
        self._last_track_update_s = data.now_s
        self._last_pitch_target_rad = pitch
        self._last_vertical_target_mps = corrected_vz
        if data.observation is not None:
            self.last_tracking_descent_velocity_mps = corrected_vz
        vertical_force = self.config.hover_thrust_n + self.vertical_velocity_pid.update(
            corrected_vz - data.barometer.vertical_velocity_mps,
            0.0,
        )
        # Keep world-vertical lift constant while the rotor disk tilts forward.
        # Compensate for the attitude the vehicle actually has, not only the
        # target attitude. This preserves vertical lift during pitch lag.
        thrust = vertical_force / max(cos(data.measured_pitch_rad), 0.5)
        command = GuidanceCommand(self.phase, thrust, pitch, trajectory, vertical_velocity_target_mps=corrected_vz)
        self.last_command = command
        return command

    def _update_acquisition_state(self, observation: TtcObservation | None) -> None:
        """Latch boost until distinct TTC samples show sustained approach."""
        if observation is None or observation is self._last_processed_observation:
            return
        self._last_processed_observation = observation
        self._last_valid_tracking_ttc_s = observation.ttc_s
        if self._acquisition_ttc_baseline_s is None:
            self._acquisition_ttc_baseline_s = observation.ttc_s
            return
        release_ttc_s = self._acquisition_ttc_baseline_s * (1.0 - TTC_PITCH_RELEASE_FRACTION)
        if observation.ttc_s <= release_ttc_s:
            self._acquisition_release_samples += 1
        else:
            self._acquisition_release_samples = 0
        if self._acquisition_release_samples >= TTC_PITCH_RELEASE_SAMPLES:
            self._acquisition_boost_active = False

    @staticmethod
    def _slew(previous: float, desired: float, maximum_change: float) -> float:
        """Move a scalar target toward its desired value without stepping."""
        return max(previous - maximum_change, min(previous + maximum_change, desired))

    def _corrected_vertical_velocity(self, trajectory: TrajectoryCommand, altitude_m: float) -> float:
        """Return the bounded descent target after altitude-error correction."""
        return max(
            -self.config.max_descent_velocity_mps,
            min(
                self.config.max_climb_velocity_mps,
                trajectory.vertical_velocity_mps
                + self.config.vertical_position_correction * (trajectory.altitude_target_m - altitude_m),
            ),
        )
