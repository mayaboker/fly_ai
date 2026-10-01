"""Small reusable PID controller for the course flight examples."""

from dataclasses import dataclass


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integral_limit: float = 0.2
    integral: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update_terms(self, error: float, rate: float, dt: float = 1 / 120) -> tuple[float, float, float]:
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral + error * dt))
        return self.kp * error, self.ki * self.integral, -self.kd * rate

    def update(self, error: float, rate: float, dt: float = 1 / 120) -> float:
        return sum(self.update_terms(error, rate, dt))
