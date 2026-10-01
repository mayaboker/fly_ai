"""Hardware-shaped vertical readings and state fusion used by TTC guidance."""

from dataclasses import dataclass

from .mission import MissionConfig


@dataclass(frozen=True)
class BarometerReading:
    """One raw and filtered barometer measurement for the guidance cycle."""

    altitude_m: float
    vertical_velocity_mps: float
    raw_altitude_m: float | None = None


@dataclass(frozen=True)
class VerticalImuReading:
    """One noisy world-up linear-acceleration sample for vertical fusion."""

    vertical_acceleration_mps2: float


@dataclass(frozen=True)
class VerticalEstimate:
    """Fused world-up height and vertical speed supplied to guidance."""

    altitude_m: float
    vertical_velocity_mps: float


class VerticalEstimator:
    """Predict vertical state from IMU acceleration and correct it with barometer altitude."""

    def __init__(self, config: MissionConfig, initial_altitude_m: float) -> None:
        self.config = config
        self.altitude_m = initial_altitude_m
        self.vertical_velocity_mps = 0.0

    def update(self, imu: VerticalImuReading, dt_s: float, barometer: BarometerReading | None = None) -> VerticalEstimate:
        """Propagate with IMU, then apply an alpha-beta barometer correction when available."""
        self.vertical_velocity_mps += imu.vertical_acceleration_mps2 * dt_s
        self.altitude_m += self.vertical_velocity_mps * dt_s
        if barometer is not None:
            error_m = barometer.altitude_m - self.altitude_m
            self.altitude_m += self.config.vertical_estimator_alpha * error_m
            self.vertical_velocity_mps += self.config.vertical_estimator_beta * error_m / dt_s
        return VerticalEstimate(self.altitude_m, self.vertical_velocity_mps)
