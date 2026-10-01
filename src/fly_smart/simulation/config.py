"""Typed simulation and field-runtime configuration for the TTC strike."""

from dataclasses import dataclass, field, replace
from math import radians

from .drone_model import DroneModel, PhysicsSettings, load_drone_profile


@dataclass(frozen=True)
class SimulationConfig:
    """Simulator scene, vehicle model, synthetic sensors, and display defaults."""

    launch_position: tuple[float, float, float] = (-5.25, 0.0, 0.05)
    target_center: tuple[float, float, float] = (24.75, 0.0, 1.0)
    target_size_m: float = 2.0
    drone_profile: str = "default"
    gravity_mps2: float = 9.81
    random_seed: int = 7
    post_impact_seconds: float = 3.0
    environment_size_px: tuple[int, int] = (960, 540)
    opencv_window_position_px: tuple[int, int] = (20, 80)
    plot_window_position_px: tuple[int, int] = (700, 80)
    wind_enabled: bool = True
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    air_density_kg_m3: float = 1.225
    body_drag_enabled: bool = True
    body_drag_cd_area_m2: tuple[float, float, float] | None = None
    angular_damping_enabled: bool = True
    angular_damping_nm_per_rad_s: tuple[float, float, float] | None = None
    rotor_aerodynamics_enabled: bool = False
    propeller_diameter_m: float | None = None
    inflow_coefficient: float = 0.35
    blade_flapping_coefficient: float = 0.10
    ground_effect_enabled: bool = False
    ground_effect_height_m: float = 0.35
    ground_effect_coefficient: float = 0.10
    ground_effect_max_multiplier: float = 1.25
    gyroscopic_torque_enabled: bool = False
    rotor_inertia_kg_m2: float | None = None

    @property
    def drone_model(self) -> DroneModel:
        """Return the selected shared drone model with URDF-derived geometry."""
        return load_drone_profile(self.drone_profile).model

    @property
    def physics_settings(self) -> PhysicsSettings:
        """Return shared physics settings resolved from this scenario's force model."""
        profile_settings = load_drone_profile(self.drone_profile).physics_settings
        return replace(
            profile_settings,
            gravity_z_mps2=-self.gravity_mps2,
            wind_enabled=self.wind_enabled,
            wind_world_mps=self.wind_world_mps,
            air_density_kg_m3=self.air_density_kg_m3,
            body_drag_enabled=self.body_drag_enabled,
            body_drag_cd_area_m2=self.body_drag_cd_area_m2 if self.body_drag_cd_area_m2 is not None else profile_settings.body_drag_cd_area_m2,
            angular_damping_enabled=self.angular_damping_enabled,
            angular_damping_nm_per_rad_s=self.angular_damping_nm_per_rad_s if self.angular_damping_nm_per_rad_s is not None else profile_settings.angular_damping_nm_per_rad_s,
            rotor_aerodynamics_enabled=self.rotor_aerodynamics_enabled,
            propeller_diameter_m=self.propeller_diameter_m if self.propeller_diameter_m is not None else profile_settings.propeller_diameter_m,
            inflow_coefficient=self.inflow_coefficient,
            blade_flapping_coefficient=self.blade_flapping_coefficient,
            ground_effect_enabled=self.ground_effect_enabled,
            ground_effect_height_m=self.ground_effect_height_m,
            ground_effect_coefficient=self.ground_effect_coefficient,
            ground_effect_max_multiplier=self.ground_effect_max_multiplier,
            gyroscopic_torque_enabled=self.gyroscopic_torque_enabled,
            rotor_inertia_kg_m2=self.rotor_inertia_kg_m2 if self.rotor_inertia_kg_m2 is not None else profile_settings.rotor_inertia_kg_m2,
        )


@dataclass(frozen=True)
class RuntimeConfig:
    """Physical camera setup, mission targets, estimator, and controller tuning."""

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
    max_descent_velocity_mps: float = 4.5
    max_climb_velocity_mps: float = 3.0
    min_ttc_s: float = 0.2
    commit_box_height_fraction: float = 0.1
    ttc_alpha: float = 0.85
    ttc_beta: float = 0.05
    min_growth_px_per_s: float = 0.01
    ttc_unavailable_descent_velocity_mps: float = 1.5
    ttc_unavailable_pitch_boost_deg: float = 0.0
    barometer_sample_hz: float = 40.0
    barometer_noise_sigma_m: float = 0.10
    barometer_bias_m: float = 0.0
    barometer_drift_sigma_m_per_sqrt_s: float = 0.0
    barometer_altitude_old_weight: float = 0.80
    barometer_velocity_old_weight: float = 0.95
    imu_sample_hz: float = 240.0
    accelerometer_noise_sigma_mps2: float = 0.0075
    accelerometer_initial_bias_sigma_mps2: float = 0.0981
    accelerometer_bias_random_walk_mps2_per_sqrt_s: float = 0.0049
    vertical_estimator_alpha: float = 0.08
    vertical_estimator_beta: float = 0.005
    altitude_pid_gains: tuple[float, float, float] = (1.8, 0.05, 2.2)
    altitude_integral_limit: float = 0.5
    forward_speed_pid_gains: tuple[float, float, float] = (0.03, 0.0, 0.002)
    forward_pitch_integral_limit: float = 0.2
    pitch_attitude_pid_gains: tuple[float, float, float] = (0.008, 0.0, 0.006)
    max_pitch_deg: float = 20.0
    vertical_velocity_pid_gains: tuple[float, float, float] = (1.0, 0.0, 0.0)
    vertical_position_correction: float = 0.8
    takeoff_altitude_tolerance_m: float = 0.2
    takeoff_velocity_tolerance_mps: float = 0.5
    commit_timeout_margin_s: float = 5.0

    @property
    def hover_thrust_n(self) -> float:
        return self.vehicle_mass_kg * self.gravity_mps2

    @property
    def commit_box_height_px(self) -> float:
        return self.camera_height_px * self.commit_box_height_fraction


@dataclass(frozen=True)
class StrikeConfig:
    """Composed application configuration with independent default groups."""

    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)

    @property
    def hover_thrust_n(self) -> float:
        return self.simulation.drone_model.mass_kg * self.simulation.gravity_mps2

    @property
    def nominal_pitch_rad(self) -> float:
        return radians(self.runtime.nominal_pitch_deg)

    @property
    def max_pitch_rad(self) -> float:
        return radians(self.runtime.max_pitch_deg)

    @property
    def ttc_unavailable_pitch_boost_rad(self) -> float:
        return radians(self.runtime.ttc_unavailable_pitch_boost_deg)

    @property
    def commit_box_height_px(self) -> float:
        return self.runtime.camera_height_px * self.runtime.commit_box_height_fraction

    def __getattr__(self, name: str):
        """Keep the old flat access API while callers migrate to grouped config."""
        for group in (self.simulation, self.runtime):
            if hasattr(group, name):
                return getattr(group, name)
        raise AttributeError(name)


# Compatibility name for code that used SceneConfig for simulator-only data.
SceneConfig = SimulationConfig
