"""Attitude-control composition above the shared physics engine."""

from math import pi

from .pid import PID


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the range from negative pi to positive pi radians."""
    return (angle + pi) % (2 * pi) - pi


class AttitudeController:
    """Turn roll, pitch, and yaw targets into body-torque requests."""

    def __init__(self, pitch_gains: tuple[float, float, float] | None = None) -> None:
        """Create default roll/yaw controllers and an optional pitch-specific tune."""
        self.roll_pid = PID(0.002, 0.0, 0.001)
        self.pitch_pid = PID(*(pitch_gains or (0.002, 0.0, 0.001)))
        self.yaw_pid = PID(0.001, 0.0, 0.0005)

    def reset(self) -> None:
        """Clear all PID history after a reset or controller-mode change."""
        for controller in (self.roll_pid, self.pitch_pid, self.yaw_pid):
            controller.reset()

    def update(self, imu: object, yaw_target: float, roll_target: float = 0.0, pitch_target: float = 0.0) -> tuple[float, float, float]:
        """Return roll, pitch, and yaw torque corrections from ideal IMU data."""
        roll, pitch, yaw = imu.roll_pitch_yaw_rad
        roll_rate, pitch_rate, yaw_rate = imu.angular_velocity_body_rad_s
        return (
            self.roll_pid.update(roll_target - roll, roll_rate),
            self.pitch_pid.update(pitch_target - pitch, pitch_rate),
            self.yaw_pid.update(wrap_angle(yaw_target - yaw), yaw_rate),
        )
