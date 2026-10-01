"""Absolute-deadline wall-clock pacing for real-time simulation runs."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


@dataclass(frozen=True)
class PacingResult:
    """Timing outcome from waiting for one physics-step deadline."""

    requested_sleep_s: float
    actual_sleep_s: float
    lateness_s: float
    rebased: bool


class RealTimePacer:
    """Keep fixed simulation steps aligned with an absolute monotonic clock."""

    def __init__(
        self,
        time_step_s: float,
        max_lag_s: float = 0.250,
        clock: Callable[[], float] = time.perf_counter,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        if time_step_s <= 0.0:
            raise ValueError("time_step_s must be positive")
        if max_lag_s < 0.0:
            raise ValueError("max_lag_s must be non-negative")
        self.time_step_s = time_step_s
        self.max_lag_s = max_lag_s
        self._clock = clock
        self._sleeper = sleeper
        self._deadline_s: float | None = None
        self.rebase_count = 0

    @property
    def active(self) -> bool:
        """Return whether a pacing origin has been established."""
        return self._deadline_s is not None

    def start(self) -> None:
        """Anchor the next deadline to the current wall clock."""
        self._deadline_s = self._clock()

    def reset(self) -> None:
        """Discard the deadline after restart or while waiting for an operator."""
        self._deadline_s = None

    def wait(self) -> PacingResult:
        """Sleep only for the remaining step budget and report deadline error."""
        if self._deadline_s is None:
            self.start()
        assert self._deadline_s is not None
        self._deadline_s += self.time_step_s
        before = self._clock()
        requested = max(0.0, self._deadline_s - before)
        if requested > 0.0:
            self._sleeper(requested)
        after = self._clock()
        actual_sleep = max(0.0, after - before)
        lateness = max(0.0, after - self._deadline_s)
        rebased = lateness > self.max_lag_s
        if rebased:
            self._deadline_s = after
            self.rebase_count += 1
        return PacingResult(requested, actual_sleep, lateness, rebased)
