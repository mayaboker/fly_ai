"""YAML adapter for the typed TTC strike configuration."""

from dataclasses import replace
from math import isfinite
from numbers import Real
from pathlib import Path

import yaml

from .config import RuntimeConfig, SimulationConfig, StrikeConfig


def _mapping(value: object, name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _position(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    if any(isinstance(item, bool) or not isinstance(item, Real) or not isfinite(float(item)) for item in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(item) for item in value)


def _gains(value: object, name: str) -> tuple[float, float, float]:
    return _position(value, name)


def _boolean(value: object, name: str) -> bool:
    """Return one YAML boolean and reject numeric lookalikes."""
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be true or false")
    return value


def _unit_interval(value: object, name: str) -> float:
    """Return one finite filter gain in the inclusive range zero to one."""
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(float(value)) or not 0.0 <= float(value) <= 1.0:
        raise ValueError(f"{name} must be a finite number from 0 to 1")
    return float(value)


def _positive(value: object, name: str) -> float:
    """Return one positive finite physical rate."""
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(float(value)) or float(value) <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return float(value)


def _non_negative(value: object, name: str) -> float:
    """Return one non-negative finite physical magnitude."""
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(float(value)) or float(value) < 0.0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return float(value)


def _finite(value: object, name: str) -> float:
    """Return one finite signed physical value."""
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _merge(instance, values: dict, names: tuple[str, ...], section_name: str):
    unknown = set(values) - set(names)
    if unknown:
        raise ValueError(f"unknown {section_name} setting(s): {', '.join(sorted(unknown))}")
    return replace(instance, **{name: values[name] for name in names if name in values})


def load_yaml_config(path: Path) -> StrikeConfig:
    """Load grouped YAML and apply only supported scenario overrides."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("the YAML root must be a mapping")

    simulation_data = _mapping(data.get("simulation"), "simulation")
    runtime_data = _mapping(data.get("runtime"), "runtime")
    if not simulation_data and not runtime_data:
        raise ValueError("YAML must contain simulation and/or runtime sections")

    scene = _mapping(simulation_data.get("scene"), "simulation.scene")
    vehicle = _mapping(simulation_data.get("vehicle_model"), "simulation.vehicle_model")
    sensor_model = _mapping(simulation_data.get("sensor_model"), "simulation.sensor_model")
    display = _mapping(simulation_data.get("display"), "simulation.display")
    recording = _mapping(simulation_data.get("recording"), "simulation.recording")
    physical = _mapping(runtime_data.get("physical_setup"), "runtime.physical_setup")
    camera = _mapping(physical.get("camera"), "runtime.physical_setup.camera")
    mission = _mapping(runtime_data.get("mission"), "runtime.mission")
    takeoff = _mapping(runtime_data.get("takeoff"), "runtime.takeoff")
    limits = _mapping(runtime_data.get("flight_limits"), "runtime.flight_limits")
    ttc = _mapping(runtime_data.get("ttc"), "runtime.ttc")
    sensors = _mapping(runtime_data.get("sensors"), "runtime.sensors")
    barometer = _mapping(sensors.get("barometer"), "runtime.sensors.barometer")
    imu = _mapping(sensors.get("imu"), "runtime.sensors.imu")
    estimator = _mapping(runtime_data.get("vertical_estimator"), "runtime.vertical_estimator")
    pid = _mapping(runtime_data.get("pid"), "runtime.pid")
    vertical = _mapping(runtime_data.get("vertical_control"), "runtime.vertical_control")
    forces = _mapping(simulation_data.get("physical_forces"), "simulation.physical_forces")
    body_drag = _mapping(forces.get("body_drag"), "simulation.physical_forces.body_drag")
    angular_damping = _mapping(forces.get("angular_damping"), "simulation.physical_forces.angular_damping")
    wind = _mapping(forces.get("wind"), "simulation.physical_forces.wind")
    rotor_aerodynamics = _mapping(forces.get("rotor_aerodynamics"), "simulation.physical_forces.rotor_aerodynamics")
    ground_effect = _mapping(forces.get("ground_effect"), "simulation.physical_forces.ground_effect")
    gyroscopic = _mapping(forces.get("gyroscopic"), "simulation.physical_forces.gyroscopic")

    simulation = SimulationConfig()
    runtime = RuntimeConfig()
    if "launch_position" in scene:
        simulation = replace(simulation, launch_position=_position(scene["launch_position"], "simulation.scene.launch_position"))
    if "target_center" in scene:
        simulation = replace(simulation, target_center=_position(scene["target_center"], "simulation.scene.target_center"))
    if "environment_size_px" in display:
        simulation = replace(simulation, environment_size_px=tuple(display["environment_size_px"]))
    if "opencv_window_position_px" in display:
        simulation = replace(simulation, opencv_window_position_px=tuple(display["opencv_window_position_px"]))
    if "plot_window_position_px" in display:
        simulation = replace(simulation, plot_window_position_px=tuple(display["plot_window_position_px"]))
    if "target_size_m" in scene:
        simulation = replace(simulation, target_size_m=scene["target_size_m"])
    if "profile" in vehicle:
        if not isinstance(vehicle["profile"], str) or not vehicle["profile"]:
            raise ValueError("simulation.vehicle_model.profile must be a non-empty profile name")
        simulation = replace(simulation, drone_profile=vehicle["profile"])
    simulation = _merge(simulation, {name: value for name, value in vehicle.items() if name != "profile"}, ("gravity_mps2",), "simulation.vehicle_model")
    simulation = _merge(simulation, sensor_model, ("random_seed",), "simulation.sensor_model")
    simulation = _merge(simulation, recording, ("post_impact_seconds",), "simulation.recording")

    if "enabled" in body_drag:
        simulation = replace(simulation, body_drag_enabled=_boolean(body_drag["enabled"], "simulation.physical_forces.body_drag.enabled"))
    if "cd_area_m2" in body_drag:
        simulation = replace(simulation, body_drag_cd_area_m2=_position(body_drag["cd_area_m2"], "simulation.physical_forces.body_drag.cd_area_m2"))
    if "air_density_kg_m3" in body_drag:
        simulation = replace(simulation, air_density_kg_m3=body_drag["air_density_kg_m3"])
    unknown = set(body_drag) - {"enabled", "cd_area_m2", "air_density_kg_m3"}
    if unknown:
        raise ValueError(f"unknown simulation.physical_forces.body_drag setting(s): {', '.join(sorted(unknown))}")

    if "enabled" in angular_damping:
        simulation = replace(simulation, angular_damping_enabled=_boolean(angular_damping["enabled"], "simulation.physical_forces.angular_damping.enabled"))
    if "coefficients_nm_per_rad_s" in angular_damping:
        simulation = replace(simulation, angular_damping_nm_per_rad_s=_position(angular_damping["coefficients_nm_per_rad_s"], "simulation.physical_forces.angular_damping.coefficients_nm_per_rad_s"))
    unknown = set(angular_damping) - {"enabled", "coefficients_nm_per_rad_s"}
    if unknown:
        raise ValueError(f"unknown simulation.physical_forces.angular_damping setting(s): {', '.join(sorted(unknown))}")

    if "enabled" in wind:
        simulation = replace(simulation, wind_enabled=_boolean(wind["enabled"], "simulation.physical_forces.wind.enabled"))
    if "world_velocity_mps" in wind:
        simulation = replace(simulation, wind_world_mps=_position(wind["world_velocity_mps"], "simulation.physical_forces.wind.world_velocity_mps"))
    unknown = set(wind) - {"enabled", "world_velocity_mps"}
    if unknown:
        raise ValueError(f"unknown simulation.physical_forces.wind setting(s): {', '.join(sorted(unknown))}")

    simulation = _merge(simulation, {name: value for name, value in rotor_aerodynamics.items() if name != "enabled"}, ("propeller_diameter_m", "inflow_coefficient", "blade_flapping_coefficient"), "simulation.physical_forces.rotor_aerodynamics")
    if "enabled" in rotor_aerodynamics:
        simulation = replace(simulation, rotor_aerodynamics_enabled=_boolean(rotor_aerodynamics["enabled"], "simulation.physical_forces.rotor_aerodynamics.enabled"))
    simulation = _merge(simulation, {name: value for name, value in ground_effect.items() if name != "enabled"}, ("ground_effect_height_m", "ground_effect_coefficient", "ground_effect_max_multiplier"), "simulation.physical_forces.ground_effect")
    if "enabled" in ground_effect:
        simulation = replace(simulation, ground_effect_enabled=_boolean(ground_effect["enabled"], "simulation.physical_forces.ground_effect.enabled"))
    simulation = _merge(simulation, {name: value for name, value in gyroscopic.items() if name != "enabled"}, ("rotor_inertia_kg_m2",), "simulation.physical_forces.gyroscopic")
    if "enabled" in gyroscopic:
        simulation = replace(simulation, gyroscopic_torque_enabled=_boolean(gyroscopic["enabled"], "simulation.physical_forces.gyroscopic.enabled"))

    runtime = _merge(runtime, camera, ("camera_width_px", "camera_height_px", "camera_hz", "camera_fov_deg", "camera_look_down_deg"), "runtime.physical_setup.camera")
    runtime = _merge(runtime, mission, ("takeoff_altitude_m", "impact_altitude_m", "forward_speed_mps", "nominal_pitch_deg"), "runtime.mission")
    runtime = _merge(runtime, takeoff, ("takeoff_max_climb_velocity_mps",), "runtime.takeoff")
    runtime = _merge(runtime, limits, ("max_descent_velocity_mps", "max_climb_velocity_mps", "max_pitch_deg", "takeoff_altitude_tolerance_m", "takeoff_velocity_tolerance_mps", "commit_timeout_margin_s"), "runtime.flight_limits")
    if "alpha" in ttc:
        runtime = replace(runtime, ttc_alpha=_unit_interval(ttc["alpha"], "runtime.ttc.alpha"))
    if "beta" in ttc:
        runtime = replace(runtime, ttc_beta=_unit_interval(ttc["beta"], "runtime.ttc.beta"))
    if "ttc_unavailable_descent_velocity_mps" in ttc:
        runtime = replace(runtime, ttc_unavailable_descent_velocity_mps=_non_negative(ttc["ttc_unavailable_descent_velocity_mps"], "runtime.ttc.ttc_unavailable_descent_velocity_mps"))
    if "ttc_unavailable_pitch_boost_deg" in ttc:
        runtime = replace(runtime, ttc_unavailable_pitch_boost_deg=_non_negative(ttc["ttc_unavailable_pitch_boost_deg"], "runtime.ttc.ttc_unavailable_pitch_boost_deg"))
    runtime = _merge(runtime, {name: value for name, value in ttc.items() if name not in {"alpha", "beta", "ttc_unavailable_descent_velocity_mps", "ttc_unavailable_pitch_boost_deg"}}, ("min_ttc_s", "commit_box_height_fraction", "min_growth_px_per_s"), "runtime.ttc")
    if "sample_hz" in barometer:
        runtime = replace(runtime, barometer_sample_hz=_positive(barometer["sample_hz"], "runtime.sensors.barometer.sample_hz"))
    if "altitude_noise_sigma_m" in barometer:
        runtime = replace(runtime, barometer_noise_sigma_m=_non_negative(barometer["altitude_noise_sigma_m"], "runtime.sensors.barometer.altitude_noise_sigma_m"))
    if "altitude_bias_m" in barometer:
        runtime = replace(runtime, barometer_bias_m=_finite(barometer["altitude_bias_m"], "runtime.sensors.barometer.altitude_bias_m"))
    if "drift_sigma_m_per_sqrt_s" in barometer:
        runtime = replace(runtime, barometer_drift_sigma_m_per_sqrt_s=_non_negative(barometer["drift_sigma_m_per_sqrt_s"], "runtime.sensors.barometer.drift_sigma_m_per_sqrt_s"))
    if "altitude_old_weight" in barometer:
        runtime = replace(runtime, barometer_altitude_old_weight=_unit_interval(barometer["altitude_old_weight"], "runtime.sensors.barometer.altitude_old_weight"))
    if "velocity_old_weight" in barometer:
        runtime = replace(runtime, barometer_velocity_old_weight=_unit_interval(barometer["velocity_old_weight"], "runtime.sensors.barometer.velocity_old_weight"))
    unknown = set(barometer) - {"sample_hz", "altitude_noise_sigma_m", "altitude_bias_m", "drift_sigma_m_per_sqrt_s", "altitude_old_weight", "velocity_old_weight"}
    if unknown:
        raise ValueError(f"unknown runtime.sensors.barometer setting(s): {', '.join(sorted(unknown))}")
    if "sample_hz" in imu:
        runtime = replace(runtime, imu_sample_hz=_positive(imu["sample_hz"], "runtime.sensors.imu.sample_hz"))
    for yaml_name, field_name in (
        ("accelerometer_noise_sigma_mps2", "accelerometer_noise_sigma_mps2"),
        ("accelerometer_initial_bias_sigma_mps2", "accelerometer_initial_bias_sigma_mps2"),
        ("accelerometer_bias_random_walk_mps2_per_sqrt_s", "accelerometer_bias_random_walk_mps2_per_sqrt_s"),
    ):
        if yaml_name in imu:
            runtime = replace(runtime, **{field_name: _non_negative(imu[yaml_name], f"runtime.sensors.imu.{yaml_name}")})
    unknown = set(imu) - {"sample_hz", "accelerometer_noise_sigma_mps2", "accelerometer_initial_bias_sigma_mps2", "accelerometer_bias_random_walk_mps2_per_sqrt_s"}
    if unknown:
        raise ValueError(f"unknown runtime.sensors.imu setting(s): {', '.join(sorted(unknown))}")
    if "alpha" in estimator:
        runtime = replace(runtime, vertical_estimator_alpha=_unit_interval(estimator["alpha"], "runtime.vertical_estimator.alpha"))
    if "beta" in estimator:
        runtime = replace(runtime, vertical_estimator_beta=_unit_interval(estimator["beta"], "runtime.vertical_estimator.beta"))
    unknown = set(estimator) - {"alpha", "beta"}
    if unknown:
        raise ValueError(f"unknown runtime.vertical_estimator setting(s): {', '.join(sorted(unknown))}")
    runtime = _merge(runtime, vertical, ("vertical_position_correction",), "runtime.vertical_control")

    for name, field_name in (("altitude", "altitude_pid_gains"), ("forward_speed", "forward_speed_pid_gains"), ("pitch_attitude", "pitch_attitude_pid_gains"), ("vertical_velocity", "vertical_velocity_pid_gains")):
        values = _mapping(pid.get(name), f"runtime.pid.{name}")
        if "gains" in values:
            runtime = replace(runtime, **{field_name: _gains(values["gains"], f"runtime.pid.{name}.gains")})
        if name == "altitude" and "integral_limit" in values:
            runtime = replace(runtime, altitude_integral_limit=values["integral_limit"])
        if name == "forward_speed" and "integral_limit" in values:
            runtime = replace(runtime, forward_pitch_integral_limit=values["integral_limit"])
        unknown = set(values) - {"gains", "integral_limit"}
        if unknown:
            raise ValueError(f"unknown runtime.pid.{name} setting(s): {', '.join(sorted(unknown))}")
    return StrikeConfig(simulation=simulation, runtime=runtime)
