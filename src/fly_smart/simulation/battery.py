"""Calibrated battery state and voltage sag for the shared drone engine."""

from dataclasses import dataclass
from math import isfinite, sqrt


def _unit_interval(value: float, name: str) -> float:
    """Validate and return one finite fraction from zero through one."""
    if not isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be a finite number from 0 to 1")
    return value


@dataclass(frozen=True)
class BatterySpec:
    """Profile-owned electrical calibration for one motor, propeller, and pack combination."""

    cell_count: int = 4
    cell_voltage_full_v: float = 4.2
    cell_voltage_nominal_v: float = 3.7
    cell_voltage_empty_v: float = 3.3
    capacity_ah: float = 1.5
    internal_resistance_ohm: float = 0.025
    max_discharge_current_a: float = 120.0
    full_throttle_current_per_motor_a: float = 30.0

    def __post_init__(self) -> None:
        """Reject incomplete or physically inverted pack settings at the profile boundary."""
        values = (
            self.cell_voltage_full_v,
            self.cell_voltage_nominal_v,
            self.cell_voltage_empty_v,
            self.capacity_ah,
            self.internal_resistance_ohm,
            self.max_discharge_current_a,
            self.full_throttle_current_per_motor_a,
        )
        if not isinstance(self.cell_count, int) or isinstance(self.cell_count, bool) or self.cell_count <= 0:
            raise ValueError("battery cell_count must be a positive integer")
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError("battery voltages, capacity, resistance, and currents must be positive finite values")
        if not self.cell_voltage_full_v > self.cell_voltage_nominal_v > self.cell_voltage_empty_v:
            raise ValueError("battery cell voltages must satisfy full > nominal > empty")

    @property
    def nominal_voltage_v(self) -> float:
        """Return the reference pack voltage used to calibrate the motor thrust curve."""
        return self.cell_count * self.cell_voltage_nominal_v


@dataclass(frozen=True)
class BatteryState:
    """One pack state reported after a physics tick."""

    state_of_charge: float
    open_circuit_voltage_v: float
    bus_voltage_v: float
    demand_current_a: float
    delivered_current_a: float
    current_limited: bool
    motor_command_scale: float


class BatteryModel:
    """Advance a calibrated battery pack from motor command fractions without electrical-motor detail."""

    def __init__(self, spec: BatterySpec) -> None:
        """Create a full battery pack with the supplied vehicle-profile specification."""
        self.spec = spec
        self.reset()

    def reset(self, state_of_charge: float = 1.0) -> None:
        """Restore the pack to one validated initial state of charge."""
        self.state_of_charge = _unit_interval(state_of_charge, "state_of_charge")

    def step(self, motor_command_fractions: tuple[float, float, float, float], time_step_s: float, bus_voltage_v: float | None = None) -> BatteryState:
        """Estimate current, advance charge, and return the voltage available to four motor commands."""
        if not isfinite(time_step_s) or time_step_s <= 0.0:
            raise ValueError("battery time_step_s must be a positive finite number")
        fractions = tuple(_unit_interval(value, "motor command fraction") for value in motor_command_fractions)
        demand_current_a = sum(self.spec.full_throttle_current_per_motor_a * fraction**2 for fraction in fractions)
        delivered_current_a = min(demand_current_a, self.spec.max_discharge_current_a)
        current_limited = demand_current_a > self.spec.max_discharge_current_a
        command_scale = sqrt(delivered_current_a / demand_current_a) if demand_current_a else 1.0
        open_circuit_voltage_v = self._open_circuit_voltage()

        if bus_voltage_v is None:
            self.state_of_charge = max(0.0, self.state_of_charge - delivered_current_a * time_step_s / (self.spec.capacity_ah * 3600.0))
            open_circuit_voltage_v = self._open_circuit_voltage()
            resolved_bus_voltage_v = max(0.0, open_circuit_voltage_v - delivered_current_a * self.spec.internal_resistance_ohm)
        else:
            if not isfinite(bus_voltage_v) or bus_voltage_v <= 0.0:
                raise ValueError("bus_voltage_v must be a positive finite number")
            resolved_bus_voltage_v = bus_voltage_v

        return BatteryState(
            self.state_of_charge,
            open_circuit_voltage_v,
            resolved_bus_voltage_v,
            demand_current_a,
            delivered_current_a,
            current_limited,
            command_scale,
        )

    def _open_circuit_voltage(self) -> float:
        """Interpolate pack open-circuit voltage from current state of charge."""
        cell_voltage_v = self.spec.cell_voltage_empty_v + self.state_of_charge * (self.spec.cell_voltage_full_v - self.spec.cell_voltage_empty_v)
        return self.spec.cell_count * cell_voltage_v
