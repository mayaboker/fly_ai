"""TTC-synchronised altitude planning without metric target range."""

from dataclasses import dataclass

from .mission import MissionConfig


@dataclass(frozen=True)
class TrajectoryCommand:
    forward_velocity_mps: float
    vertical_velocity_mps: float
    altitude_target_m: float


class TtcDescentPlanner:
    """Use TTC as time-to-go for reaching the known impact altitude."""

    def __init__(self, config: MissionConfig) -> None:
        self.config = config

    def command(self, ttc_s: float | None, current_altitude_m: float) -> TrajectoryCommand:
        """Return nominal forward motion and a TTC-synchronised vertical target.

        Target size and distance are intentionally absent. TTC supplies only
        the available time; the barometer supplies current altitude and the
        mission supplies the altitude at which contact should occur.
        """
        altitude_error_m = self.config.impact_altitude_m - current_altitude_m
        if ttc_s is None:
            # Start shedding takeoff overshoot before visual TTC is reliable.
            vertical_velocity_mps = -self.config.ttc_unavailable_descent_velocity_mps
            altitude_target_m = self.config.takeoff_altitude_m
        else:
            time_to_go_s = max(ttc_s, self.config.min_ttc_s)
            vertical_velocity_mps = altitude_error_m / time_to_go_s
            altitude_target_m = self.config.impact_altitude_m
        vertical_velocity_mps = max(
            -self.config.max_descent_velocity_mps,
            min(self.config.max_climb_velocity_mps, vertical_velocity_mps),
        )
        return TrajectoryCommand(
            self.config.forward_speed_mps,
            vertical_velocity_mps,
            altitude_target_m,
        )
