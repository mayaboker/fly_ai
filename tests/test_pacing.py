"""Tests for absolute-deadline real-time simulation pacing."""

import pytest

from fly_smart.simulation.pacing import RealTimePacer


class _Clock:
    """Controllable monotonic clock whose sleeper advances virtual time."""

    def __init__(self) -> None:
        self.value = 0.0
        self.sleeps: list[float] = []

    def now(self) -> float:
        return self.value

    def sleep(self, duration: float) -> None:
        self.sleeps.append(duration)
        self.value += duration


def test_pacer_sleeps_only_for_remaining_budget_and_recovers_overrun():
    clock = _Clock()
    pacer = RealTimePacer(0.004, clock=clock.now, sleeper=clock.sleep)
    pacer.start()
    clock.value += 0.001

    first = pacer.wait()

    assert first.requested_sleep_s == pytest.approx(0.003)
    assert first.lateness_s == pytest.approx(0.0)
    clock.value += 0.006
    overrun = pacer.wait()
    assert overrun.requested_sleep_s == 0.0
    assert overrun.lateness_s == pytest.approx(0.002)
    clock.value += 0.001
    recovered = pacer.wait()
    assert recovered.requested_sleep_s == pytest.approx(0.001)


def test_pacer_rebases_after_long_stall_and_reset_starts_fresh():
    clock = _Clock()
    pacer = RealTimePacer(0.004, max_lag_s=0.250, clock=clock.now, sleeper=clock.sleep)
    pacer.start()
    clock.value = 0.300

    stalled = pacer.wait()

    assert stalled.rebased
    assert pacer.rebase_count == 1
    next_step = pacer.wait()
    assert next_step.requested_sleep_s == pytest.approx(0.004)
    pacer.reset()
    clock.value = 10.0
    pacer.start()
    resumed = pacer.wait()
    assert resumed.requested_sleep_s == pytest.approx(0.004)


def test_pacer_rejects_invalid_configuration():
    with pytest.raises(ValueError):
        RealTimePacer(0.0)
    with pytest.raises(ValueError):
        RealTimePacer(0.01, max_lag_s=-1.0)
