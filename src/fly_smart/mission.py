"""Configuration shared by real-flight adapters and the simulator."""

from dataclasses import dataclass
from math import radians


@dataclass(frozen=True)
class MissionConfig:
    """Mission, camera, estimator, and controller settings independent of a world model."""

    vehicle_mass_kg: float = 0.65
    gravity_mps2: float = 9.81
    camera_width_px: int = 640
    camera_height_px: int = 480
    camera_hz: int = 30
    camera_fov_deg: float = 90.0
    camera_look_down_deg: float = 0.0
    takeoff_altitude_m: float = 15.0
    takeoff_max_climb_velocity_mps: float = 7.2
    impact_altitude_m: float = 1.0
    forward_speed_mps: float = 13.0
    nominal_pitch_deg: float = 20.0
    max_pitch_deg: float = 20.0
    max_descent_velocity_mps: float = 4.5
    max_climb_velocity_mps: float = 3.0
    min_ttc_s: float = 0.2
    commit_box_height_fraction: float = 0.1
    ttc_alpha: float = 0.85
    ttc_beta: float = 0.05
    min_growth_px_per_s: float = 0.01
    ttc_unavailable_descent_velocity_mps: float = 1.5
    ttc_unavailable_pitch_boost_deg: float = 0.0
    vertical_estimator_alpha: float = 0.08
    vertical_estimator_beta: float = 0.005
    altitude_pid_gains: tuple[float, float, float] = (1.8, 0.05, 2.2)
    altitude_integral_limit: float = 0.5
    forward_speed_pid_gains: tuple[float, float, float] = (0.03, 0.0, 0.002)
    forward_pitch_integral_limit: float = 0.2
    pitch_attitude_pid_gains: tuple[float, float, float] = (0.008, 0.0, 0.006)
    vertical_velocity_pid_gains: tuple[float, float, float] = (1.0, 0.0, 0.0)
    vertical_position_correction: float = 0.8
    takeoff_altitude_tolerance_m: float = 0.2
    takeoff_velocity_tolerance_mps: float = 0.5
    commit_timeout_margin_s: float = 5.0

    @property
    def hover_thrust_n(self) -> float:
        return self.vehicle_mass_kg * self.gravity_mps2

    @property
    def nominal_pitch_rad(self) -> float:
        return radians(self.nominal_pitch_deg)

    @property
    def max_pitch_rad(self) -> float:
        return radians(self.max_pitch_deg)

    @property
    def ttc_unavailable_pitch_boost_rad(self) -> float:
        return radians(self.ttc_unavailable_pitch_boost_deg)

    @property
    def commit_box_height_px(self) -> float:
        return self.camera_height_px * self.commit_box_height_fraction
