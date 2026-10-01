"""Geometry-free bounding-box time-to-contact estimation."""

from dataclasses import dataclass
from math import sqrt

from .mission import MissionConfig


@dataclass(frozen=True)
class TtcObservation:
    box: tuple[int, int, int, int]
    scale_px: float
    raw_growth_px_s: float
    raw_ttc_s: float
    scale_growth_px_s: float
    ttc_s: float


class BboxTtcTracker:
    """Estimate TTC from bbox scale with an alpha-beta scale/rate filter."""

    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.estimated_scale_px: float | None = None
        self.last_measured_scale_px: float | None = None
        self.last_time_s: float | None = None
        self.estimated_growth_px_s = 0.0
        self.commit_ready = False
        self.last_observation: TtcObservation | None = None

    def update(self, box: tuple[int, int, int, int] | None, now_s: float) -> TtcObservation | None:
        if box is None:
            return None
        _, _, width_px, height_px = box
        scale_px = sqrt(width_px * height_px)
        self.commit_ready = self.commit_ready or height_px >= self.config.commit_box_height_px
        if self.last_time_s is None:
            self.estimated_scale_px = scale_px
            self.last_measured_scale_px = scale_px
            self.last_time_s = now_s
            return None
        dt_s = now_s - self.last_time_s
        if dt_s <= 0:
            return None
        predicted_scale_px = self.estimated_scale_px + self.estimated_growth_px_s * dt_s
        residual_px = scale_px - predicted_scale_px
        growth_px_s = (scale_px - self.last_measured_scale_px) / dt_s
        raw_ttc_s = scale_px / growth_px_s if growth_px_s > 0.0 else float("nan")
        self.estimated_scale_px = predicted_scale_px + self.config.ttc_alpha * residual_px
        self.estimated_growth_px_s += self.config.ttc_beta * residual_px / dt_s
        self.last_measured_scale_px = scale_px
        self.last_time_s = now_s
        if self.estimated_growth_px_s <= self.config.min_growth_px_per_s:
            return None
        observation = TtcObservation(
            box,
            scale_px,
            growth_px_s,
            raw_ttc_s,
            self.estimated_growth_px_s,
            self.estimated_scale_px / self.estimated_growth_px_s,
        )
        self.last_observation = observation
        return observation
