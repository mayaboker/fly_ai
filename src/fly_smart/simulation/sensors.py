"""Seeded synthetic sensors used only by the PyBullet runner."""

import numpy as np

from .config import StrikeConfig
from ..sensing import BarometerReading, VerticalImuReading


class Barometer:
    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.period_s = 1 / config.barometer_sample_hz
        self.rng = np.random.default_rng(config.random_seed)
        self.last_time_s: float | None = None
        self.last_altitude_m: float | None = None
        self.filtered_altitude_m: float | None = None
        self.vertical_velocity_mps = 0.0
        self.drift_m = 0.0

    def sample(self, true_altitude_m: float, now_s: float) -> BarometerReading | None:
        if self.last_time_s is not None and now_s - self.last_time_s < self.period_s:
            return None
        elapsed_s = 0.0 if self.last_time_s is None else now_s - self.last_time_s
        if self.config.barometer_drift_sigma_m_per_sqrt_s:
            self.drift_m += self.rng.normal(0.0, self.config.barometer_drift_sigma_m_per_sqrt_s * elapsed_s**0.5)
        raw_altitude_m = true_altitude_m + self.config.barometer_bias_m + self.drift_m + self.rng.normal(0.0, self.config.barometer_noise_sigma_m)
        self.filtered_altitude_m = raw_altitude_m if self.filtered_altitude_m is None else self.config.barometer_altitude_old_weight * self.filtered_altitude_m + (1 - self.config.barometer_altitude_old_weight) * raw_altitude_m
        if self.last_time_s is not None and self.last_altitude_m is not None:
            measured_velocity = (self.filtered_altitude_m - self.last_altitude_m) / (now_s - self.last_time_s)
            self.vertical_velocity_mps = self.config.barometer_velocity_old_weight * self.vertical_velocity_mps + (1 - self.config.barometer_velocity_old_weight) * measured_velocity
        self.last_time_s, self.last_altitude_m = now_s, self.filtered_altitude_m
        return BarometerReading(self.filtered_altitude_m, self.vertical_velocity_mps, raw_altitude_m)


class VerticalImu:
    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.rng = np.random.default_rng(config.random_seed + 1)
        self.bias_mps2 = self.rng.normal(0.0, config.accelerometer_initial_bias_sigma_mps2)
        self.last_time_s: float | None = None

    def sample(self, true_vertical_acceleration_mps2: float, now_s: float) -> VerticalImuReading:
        elapsed_s = 0.0 if self.last_time_s is None else now_s - self.last_time_s
        if self.config.accelerometer_bias_random_walk_mps2_per_sqrt_s:
            self.bias_mps2 += self.rng.normal(0.0, self.config.accelerometer_bias_random_walk_mps2_per_sqrt_s * elapsed_s**0.5)
        self.last_time_s = now_s
        return VerticalImuReading(true_vertical_acceleration_mps2 + self.bias_mps2 + self.rng.normal(0.0, self.config.accelerometer_noise_sigma_mps2))
