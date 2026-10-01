"""Typed drone profiles, physical models, clock settings, and state."""

from dataclasses import dataclass, field, replace
from functools import lru_cache
from math import isfinite
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import yaml

from .battery import BatterySpec


@dataclass(frozen=True)
class VehicleGeometry:
    """Rigid-body and rotor geometry read from one course quadcopter URDF."""

    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    inertia_kg_m2: tuple[float, float, float, float, float, float]
    rotor_positions_m: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class PropellerSpec:
    """Listing facts for one propeller, with only diameter used by the force model."""

    name: str
    diameter_in: float
    pitch_in: float | None = None
    blade_count: int | None = None
    rotation_set: str | None = None
    hub_diameter_mm: float | None = None
    shaft_diameter_mm: float | None = None
    material: str | None = None
    weight_g: float | None = None
    package_count: int | None = None
    source: str | None = None

    def __post_init__(self) -> None:
        """Validate listing values while allowing unknown optional product fields."""
        positive_numbers = {
            "diameter_in": self.diameter_in,
            "pitch_in": self.pitch_in,
            "hub_diameter_mm": self.hub_diameter_mm,
            "shaft_diameter_mm": self.shaft_diameter_mm,
            "weight_g": self.weight_g,
        }
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("propeller name must not be empty")
        for field_name, value in positive_numbers.items():
            if value is not None and (not isfinite(value) or value <= 0.0):
                raise ValueError(f"propeller {field_name} must be a positive finite number")
        for field_name, value in {"blade_count": self.blade_count, "package_count": self.package_count}.items():
            if value is not None and (not isinstance(value, int) or isinstance(value, bool) or value <= 0):
                raise ValueError(f"propeller {field_name} must be a positive integer")

    @property
    def diameter_m(self) -> float:
        """Convert the listing's inch diameter to the SI unit used by physics."""
        return self.diameter_in * 0.0254


@dataclass(frozen=True)
class DroneModel:
    """Physical and actuator constants for the course's four-rotor drone."""

    mass_kg: float = 0.65
    center_of_mass_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia_kg_m2: tuple[float, float, float, float, float, float] = (0.00348, 0.00348, 0.00396, 0.0, 0.0, 0.0)
    rotor_positions_m: tuple[tuple[float, float, float], ...] = ((0.12, 0.12, 0.02), (0.12, -0.12, 0.02), (-0.12, 0.12, 0.02), (-0.12, -0.12, 0.02))
    motor_kv_rpm_per_v: float = 24000.0 / (4 * 3.7)
    max_thrust_per_motor_n: float = 0.65 * 9.81
    motor_time_constant_s: float = 0.05
    rotor_drag_coefficient: float = 2e-6
    motor_yaw_signs: tuple[int, int, int, int] = (1, -1, -1, 1)
    battery: BatterySpec = field(default_factory=BatterySpec)
    urdf_path: Path = Path(__file__).parent / "assets" / "full_drone.urdf"

    @property
    def max_rpm(self) -> float:
        """Return the nominal-voltage no-load RPM derived from motor KV."""
        return self.motor_kv_rpm_per_v * self.battery.nominal_voltage_v

    @property
    def thrust_coefficient(self) -> float:
        """Return the teaching-model thrust coefficient in N/RPM²."""
        return self.max_thrust_per_motor_n / self.max_rpm**2

    @property
    def torque_coefficient(self) -> float:
        """Return the reaction-torque coefficient in N m/RPM²."""
        return 0.015 * self.thrust_coefficient

    @property
    def motor_positions_m(self) -> tuple[tuple[float, float], ...]:
        """Return the four URDF-derived rotor X/Y positions in the body frame."""
        return tuple((x, y) for x, y, _ in self.rotor_positions_m)


@dataclass(frozen=True)
class PhysicsSettings:
    """World, clock, and environment settings for a PyBullet simulation."""

    gravity_z_mps2: float = -9.81
    physics_hz: int = 240
    control_hz: int = 120
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wind_enabled: bool = True
    air_density_kg_m3: float = 1.225
    body_drag_enabled: bool = True
    body_drag_cd_area_m2: tuple[float, float, float] = (0.012, 0.012, 0.020)
    angular_damping_enabled: bool = True
    angular_damping_nm_per_rad_s: tuple[float, float, float] = (0.0012, 0.0012, 0.0020)
    rotor_aerodynamics_enabled: bool = False
    propeller_diameter_m: float = 0.127
    inflow_coefficient: float = 0.35
    blade_flapping_coefficient: float = 0.10
    ground_effect_enabled: bool = False
    ground_effect_height_m: float = 0.35
    ground_effect_coefficient: float = 0.10
    ground_effect_max_multiplier: float = 1.25
    gyroscopic_torque_enabled: bool = False
    rotor_inertia_kg_m2: float = 5e-6

    @property
    def time_step_s(self) -> float:
        """Return one PyBullet integration timestep in seconds."""
        return 1.0 / self.physics_hz

    @property
    def control_steps(self) -> int:
        """Return the number of physics steps between controller updates."""
        return self.physics_hz // self.control_hz


@dataclass(frozen=True)
class DroneProfile:
    """One reusable vehicle definition: URDF mass plus actuator and aero constants."""

    name: str
    model: DroneModel
    physics_settings: PhysicsSettings
    propeller: PropellerSpec


def _profile_path(name: str) -> Path:
    """Return the YAML path for one named shared drone profile."""
    path = Path(__file__).parent / "drone_profiles" / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"unknown drone profile '{name}'")
    return path


def _vector(value: str | None, name: str) -> tuple[float, float, float]:
    """Parse one required three-value URDF vector attribute."""
    try:
        values = tuple(float(item) for item in (value or "").split())
    except ValueError as exc:
        raise ValueError(f"{name} must contain three finite numbers") from exc
    if len(values) != 3 or not all(np.isfinite(values)):
        raise ValueError(f"{name} must contain three finite numbers")
    return values


def _geometry_from_urdf(path: Path) -> VehicleGeometry:
    """Read base inertia and ordered rotor-joint geometry from a course URDF."""
    try:
        root = ElementTree.parse(path).getroot()
        inertial = root.find("./link[@name='base_link']/inertial")
        if inertial is None:
            raise ValueError("base_link needs an inertial block")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or "value" not in mass.attrib or inertia is None:
            raise ValueError("base_link needs mass and inertia")
        mass_kg = float(mass.attrib["value"])
        center_origin = inertial.find("origin")
        center_of_mass_m = _vector(center_origin.attrib.get("xyz") if center_origin is not None else "0 0 0", "base_link inertial origin")
        inertia_kg_m2 = tuple(float(inertia.attrib[name]) for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
        if not all(np.isfinite(inertia_kg_m2)) or any(value < 0.0 for value in inertia_kg_m2[:3]):
            raise ValueError("base_link inertia must be finite with non-negative diagonal values")

        rotor_positions: list[tuple[float, float, float]] = []
        for index in range(4):
            joint = root.find(f"./joint[@name='rotor_{index}_joint']")
            if joint is None or joint.attrib.get("type") != "fixed":
                raise ValueError(f"rotor_{index}_joint must be a fixed joint")
            parent, child, origin = joint.find("parent"), joint.find("child"), joint.find("origin")
            if parent is None or parent.attrib.get("link") != "base_link" or child is None or child.attrib.get("link") != f"rotor_{index}" or origin is None:
                raise ValueError(f"rotor_{index}_joint must connect base_link to rotor_{index} with an origin")
            rotor_positions.append(_vector(origin.attrib.get("xyz"), f"rotor_{index}_joint origin"))
    except (ElementTree.ParseError, ValueError) as exc:
        raise ValueError(f"invalid drone URDF '{path}': {exc}") from exc
    if mass_kg <= 0 or not np.isfinite(mass_kg):
        raise ValueError(f"invalid drone URDF '{path}': mass must be positive")
    return VehicleGeometry(mass_kg, center_of_mass_m, inertia_kg_m2, tuple(rotor_positions))


@lru_cache(maxsize=None)
def load_drone_profile(name: str = "default") -> DroneProfile:
    """Load one profile and derive its model mass from the linked URDF."""
    path = _profile_path(name)
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid drone profile '{path}': {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"drone profile '{path}' must be a mapping")

    allowed = {"name", "urdf", "actuators", "battery", "aerodynamics", "propeller"}
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f"unknown drone profile setting(s): {', '.join(sorted(unknown))}")
    actuators = data.get("actuators", {})
    battery = data.get("battery", {})
    aerodynamics = data.get("aerodynamics", {})
    propeller_data = data.get("propeller")
    if not isinstance(actuators, dict) or not isinstance(battery, dict) or not isinstance(aerodynamics, dict):
        raise ValueError(f"drone profile '{path}' actuator, battery, and aerodynamic sections must be mappings")
    if propeller_data is None:
        legacy_diameter_m = aerodynamics.get("propeller_diameter_m")
        if legacy_diameter_m is None:
            raise ValueError(f"drone profile '{path}' needs a propeller section or aerodynamics.propeller_diameter_m")
        propeller = PropellerSpec("legacy propeller", float(legacy_diameter_m) / 0.0254)
    elif isinstance(propeller_data, dict):
        propeller = PropellerSpec(**propeller_data)
    else:
        raise ValueError(f"drone profile '{path}' propeller section must be a mapping")

    urdf_name = data.get("urdf")
    if not isinstance(urdf_name, str):
        raise ValueError(f"drone profile '{path}' needs a URDF filename")
    urdf_path = Path(__file__).parent / "assets" / urdf_name
    geometry = _geometry_from_urdf(urdf_path)
    model = DroneModel(
        mass_kg=geometry.mass_kg,
        center_of_mass_m=geometry.center_of_mass_m,
        inertia_kg_m2=geometry.inertia_kg_m2,
        rotor_positions_m=geometry.rotor_positions_m,
        battery=BatterySpec(**battery),
        urdf_path=urdf_path,
        **{key: tuple(value) if key == "motor_yaw_signs" else value for key, value in actuators.items()},
    )
    if len(model.motor_yaw_signs) != len(model.rotor_positions_m):
        raise ValueError(f"drone profile '{path}' needs one motor_yaw_sign per URDF rotor")
    settings = replace(
        PhysicsSettings(),
        propeller_diameter_m=propeller.diameter_m,
        **{
            key: tuple(value) if key in {"body_drag_cd_area_m2", "angular_damping_nm_per_rad_s"} else value
            for key, value in aerodynamics.items()
            if key != "propeller_diameter_m"
        },
    )
    return DroneProfile(str(data.get("name", name)), model, settings, propeller)


@dataclass(frozen=True)
class DroneState:
    """Ideal PyBullet state: position, velocity, quaternion, and body rate."""

    position_m: tuple[float, float, float]
    linear_velocity_mps: tuple[float, float, float]
    orientation_quaternion: tuple[float, float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class ImuReading:
    """Ideal IMU attitude and body-frame angular-rate measurement."""

    roll_pitch_yaw_rad: tuple[float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class PhysicsStep:
    """Actuator, force, and state result produced by one engine step."""

    collective_pwm_us: float
    battery_state_of_charge: float
    bus_voltage_v: float
    demand_current_a: float
    delivered_current_a: float
    current_limited: bool
    available_rpm_per_motor: float
    motor_rpms: tuple[float, float, float, float]
    motor_thrusts_n: tuple[float, float, float, float]
    total_thrust_n: float
    drag_force_body_n: tuple[float, float, float]
    body_drag_force_body_n: tuple[float, float, float]
    angular_damping_torque_body_nm: tuple[float, float, float]
    gyroscopic_torque_body_nm: tuple[float, float, float]
    ground_effect_multipliers: tuple[float, float, float, float]
    state: DroneState


DEFAULT_DRONE_PROFILE = load_drone_profile()
DEFAULT_DRONE_MODEL = DEFAULT_DRONE_PROFILE.model
DEFAULT_PHYSICS_SETTINGS = DEFAULT_DRONE_PROFILE.physics_settings
