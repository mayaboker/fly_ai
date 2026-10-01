"""Motor, configurable flight forces, and PyBullet integration."""

from math import pi, sqrt

import numpy as np
import pybullet as p

from .battery import BatteryModel
from .drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, PhysicsSettings, PhysicsStep
from .pybullet_sensors import read_state, world_to_body_vector

PWM_MIN, PWM_HOVER, PWM_MAX = 1000.0, 1500.0, 2000.0


def clamp(value: float, low: float, high: float) -> float:
    """Limit a scalar to an inclusive lower and upper bound."""
    return max(low, min(high, value))


def thrust_from_pwm(pwm_us: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Map collective PWM to one motor's requested thrust in newtons."""
    normalized = clamp((pwm_us - PWM_MIN) / (PWM_MAX - PWM_MIN), 0.0, 1.0)
    return model.max_thrust_per_motor_n * normalized**2


def rpm_from_thrust(thrust_n: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Convert one motor's bounded thrust request into target RPM."""
    return sqrt(clamp(thrust_n, 0.0, model.max_thrust_per_motor_n) / model.thrust_coefficient)


def pwm_from_thrust(thrust_n: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Convert one motor's bounded thrust request into collective PWM."""
    normalized = sqrt(clamp(thrust_n, 0.0, model.max_thrust_per_motor_n) / model.max_thrust_per_motor_n)
    return PWM_MIN + (PWM_MAX - PWM_MIN) * normalized


class PhysicsEngine:
    """Own actuator state, apply all flight forces, and advance PyBullet once."""

    def __init__(self, model: DroneModel = DEFAULT_DRONE_MODEL, settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS) -> None:
        """Create an engine with one model, environment, and four stopped motors."""
        self.model = model
        self.settings = settings
        self._motor_rpms = (0.0, 0.0, 0.0, 0.0)
        self._battery = BatteryModel(model.battery)
        yaw_torque_per_newton = model.torque_coefficient / model.thrust_coefficient
        allocation = np.array(
            [
                [y for _, y, _ in model.rotor_positions_m],
                [-x for x, _, _ in model.rotor_positions_m],
                [yaw_sign * yaw_torque_per_newton for yaw_sign in model.motor_yaw_signs],
            ]
        )
        self._torque_allocation_pseudoinverse = np.linalg.pinv(allocation)

    def reset(self, initial_rpms: tuple[float, float, float, float] | None = None, state_of_charge: float = 1.0) -> None:
        """Reset motor RPM and battery charge, optionally priming RPM values for a test."""
        self._motor_rpms = initial_rpms or (0.0, 0.0, 0.0, 0.0)
        self._battery.reset(state_of_charge)

    def pwm_from_thrust(self, thrust_n: float) -> float:
        """Convert a one-motor thrust request to PWM using this engine's model."""
        return pwm_from_thrust(thrust_n, self.model)

    def thrust_from_pwm(self, pwm_us: float) -> float:
        """Convert collective PWM to one-motor requested thrust for this model."""
        return thrust_from_pwm(pwm_us, self.model)

    def step(self, drone: int, collective_pwm_us: float, body_torque_nm: tuple[float, float, float], bus_voltage_v: float | None = None) -> PhysicsStep:
        """Mix commands, resolve battery voltage, advance motors, apply forces, and integrate once."""
        requested_thrusts = self._mix_motor_thrusts(self.thrust_from_pwm(collective_pwm_us), body_torque_nm)
        nominal_target_rpms = tuple(rpm_from_thrust(thrust, self.model) for thrust in requested_thrusts)
        command_fractions = tuple(clamp(rpm / self.model.max_rpm, 0.0, 1.0) for rpm in nominal_target_rpms)
        battery_state = self._battery.step(command_fractions, self.settings.time_step_s, bus_voltage_v)
        voltage_scale = battery_state.bus_voltage_v / self.model.battery.nominal_voltage_v
        target_rpms = tuple(rpm * voltage_scale * battery_state.motor_command_scale for rpm in nominal_target_rpms)
        self._motor_rpms = self._advance_motor_rpms(target_rpms)
        state = read_state(drone)
        air_velocity_body = self._air_velocity_body(drone, state)
        motor_thrusts, total_thrust, ground_effect_multipliers = self._apply_rotor_forces(drone, air_velocity_body)
        rotor_drag_force = self._apply_rotor_drag(drone, air_velocity_body)
        body_drag_force = self._apply_body_drag(drone, air_velocity_body)
        angular_damping_torque = self._apply_angular_damping(drone, state)
        gyroscopic_torque = self._apply_gyroscopic_torque(drone, state)
        p.stepSimulation()
        drag_force = tuple(rotor + body for rotor, body in zip(rotor_drag_force, body_drag_force))
        return PhysicsStep(
            collective_pwm_us,
            battery_state.state_of_charge,
            battery_state.bus_voltage_v,
            battery_state.demand_current_a,
            battery_state.delivered_current_a,
            battery_state.current_limited,
            self.model.motor_kv_rpm_per_v * battery_state.bus_voltage_v,
            self._motor_rpms,
            motor_thrusts,
            total_thrust,
            drag_force,
            body_drag_force,
            angular_damping_torque,
            gyroscopic_torque,
            ground_effect_multipliers,
            read_state(drone),
        )

    def _mix_motor_thrusts(self, collective_thrust_n: float, torque_nm: tuple[float, float, float]) -> tuple[float, float, float, float]:
        """Mix collective and body torque while scaling corrections at motor limits."""
        roll_torque, pitch_torque, yaw_torque = torque_nm
        collective = clamp(collective_thrust_n, 0.0, self.model.max_thrust_per_motor_n)
        deltas = tuple(float(value) for value in self._torque_allocation_pseudoinverse @ np.array((roll_torque, pitch_torque, yaw_torque)))
        scale = 1.0
        for delta in deltas:
            if delta > 0:
                scale = min(scale, (self.model.max_thrust_per_motor_n - collective) / delta)
            elif delta < 0:
                scale = min(scale, collective / -delta)
        return tuple(collective + scale * delta for delta in deltas)

    def _advance_motor_rpms(self, target_rpms: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        """Advance actual RPM with the model's first-order motor response delay."""
        alpha = min(1.0, self.settings.time_step_s / self.model.motor_time_constant_s)
        return tuple(actual + alpha * (target - actual) for actual, target in zip(self._motor_rpms, target_rpms))

    def _apply_rotor_forces(self, drone: int, air_velocity_body: tuple[float, float, float]) -> tuple[tuple[float, float, float, float], float, tuple[float, float, float, float]]:
        """Apply per-rotor thrust, reaction torque, and optional rotor aerodynamics."""
        base_thrusts = tuple(self.model.thrust_coefficient * rpm**2 for rpm in self._motor_rpms)
        inflow_multiplier = self._inflow_multiplier(air_velocity_body)
        ground_effect_multipliers = tuple(self._ground_effect_multiplier(drone, index) for index in range(4))
        motor_thrusts = tuple(base * inflow_multiplier * ground for base, ground in zip(base_thrusts, ground_effect_multipliers))
        for link_index, (yaw_sign, thrust, rpm) in enumerate(zip(self.model.motor_yaw_signs, motor_thrusts, self._motor_rpms)):
            p.applyExternalForce(drone, link_index, (0, 0, thrust), (0, 0, 0), p.LINK_FRAME)
            p.applyExternalTorque(drone, -1, (0, 0, yaw_sign * self.model.torque_coefficient * rpm**2), p.LINK_FRAME)
            flapping_force = self._blade_flapping_force(thrust, rpm, air_velocity_body)
            p.applyExternalForce(drone, link_index, flapping_force, (0, 0, 0), p.LINK_FRAME)
        return motor_thrusts, sum(motor_thrusts), ground_effect_multipliers

    def _air_velocity_body(self, drone: int, state) -> tuple[float, float, float]:
        """Return velocity relative to the configured constant wind in body axes."""
        wind = self.settings.wind_world_mps if self.settings.wind_enabled else (0.0, 0.0, 0.0)
        relative_velocity_world = tuple(velocity - wind_component for velocity, wind_component in zip(state.linear_velocity_mps, wind))
        return world_to_body_vector(drone, relative_velocity_world)

    def _apply_rotor_drag(self, drone: int, air_velocity_body: tuple[float, float, float]) -> tuple[float, float, float]:
        """Apply the existing rotor-dependent linear drag opposite body-relative airspeed."""
        drag_force = tuple(-self.model.rotor_drag_coefficient * sum(self._motor_rpms) * component for component in air_velocity_body)
        p.applyExternalForce(drone, -1, drag_force, (0, 0, 0), p.LINK_FRAME)
        return drag_force

    def _apply_body_drag(self, drone: int, air_velocity_body: tuple[float, float, float]) -> tuple[float, float, float]:
        """Apply quadratic frame drag using per-axis drag-area values."""
        if not self.settings.body_drag_enabled:
            return (0.0, 0.0, 0.0)
        speed = sqrt(sum(component**2 for component in air_velocity_body))
        drag_force = tuple(
            -0.5 * self.settings.air_density_kg_m3 * cd_area * speed * velocity
            for velocity, cd_area in zip(air_velocity_body, self.settings.body_drag_cd_area_m2)
        )
        p.applyExternalForce(drone, -1, drag_force, (0, 0, 0), p.LINK_FRAME)
        return drag_force

    def _apply_angular_damping(self, drone: int, state) -> tuple[float, float, float]:
        """Apply an aerodynamic body torque that opposes angular velocity."""
        if not self.settings.angular_damping_enabled:
            return (0.0, 0.0, 0.0)
        torque = tuple(-coefficient * rate for coefficient, rate in zip(self.settings.angular_damping_nm_per_rad_s, state.angular_velocity_body_rad_s))
        p.applyExternalTorque(drone, -1, torque, p.LINK_FRAME)
        return torque

    def _inflow_multiplier(self, air_velocity_body: tuple[float, float, float]) -> float:
        """Return optional forward-flight thrust reduction from rotor advance ratio."""
        if not self.settings.rotor_aerodynamics_enabled:
            return 1.0
        horizontal_speed = sqrt(air_velocity_body[0] ** 2 + air_velocity_body[1] ** 2)
        mean_rpm = max(sum(self._motor_rpms) / 4.0, 1.0)
        tip_speed = pi * self.settings.propeller_diameter_m * mean_rpm / 60.0
        advance_ratio = horizontal_speed / max(tip_speed, 1e-6)
        return clamp(1.0 - self.settings.inflow_coefficient * advance_ratio, 0.5, 1.0)

    def _blade_flapping_force(self, thrust_n: float, rpm: float, air_velocity_body: tuple[float, float, float]) -> tuple[float, float, float]:
        """Return optional in-plane rotor force opposing horizontal body-relative airspeed."""
        if not self.settings.rotor_aerodynamics_enabled or rpm <= 0.0:
            return (0.0, 0.0, 0.0)
        tip_speed = pi * self.settings.propeller_diameter_m * rpm / 60.0
        scale = -self.settings.blade_flapping_coefficient * thrust_n / max(tip_speed, 1e-6)
        return (scale * air_velocity_body[0], scale * air_velocity_body[1], 0.0)

    def _ground_effect_multiplier(self, drone: int, link_index: int) -> float:
        """Return optional capped thrust increase from the nearest surface below one rotor."""
        if not self.settings.ground_effect_enabled:
            return 1.0
        origin = p.getLinkState(drone, link_index)[0]
        end = (origin[0], origin[1], origin[2] - self.settings.ground_effect_height_m)
        hit = p.rayTest(origin, end)[0]
        hit_fraction = hit[2]
        if hit[0] < 0 or hit_fraction >= 1.0:
            return 1.0
        height = max(hit_fraction * self.settings.ground_effect_height_m, 0.01)
        ratio = self.settings.propeller_diameter_m / (4.0 * height)
        multiplier = 1.0 + self.settings.ground_effect_coefficient * ratio**2
        return clamp(multiplier, 1.0, self.settings.ground_effect_max_multiplier)

    def _apply_gyroscopic_torque(self, drone: int, state) -> tuple[float, float, float]:
        """Apply optional body torque from the signed rotor angular momentum."""
        if not self.settings.gyroscopic_torque_enabled:
            return (0.0, 0.0, 0.0)
        angular_momentum_z = sum(
            yaw_sign * self.settings.rotor_inertia_kg_m2 * rpm * 2.0 * pi / 60.0
            for yaw_sign, rpm in zip(self.model.motor_yaw_signs, self._motor_rpms)
        )
        roll_rate, pitch_rate, _ = state.angular_velocity_body_rad_s
        torque = (pitch_rate * angular_momentum_z, -roll_rate * angular_momentum_z, 0.0)
        p.applyExternalTorque(drone, -1, torque, p.LINK_FRAME)
        return torque
