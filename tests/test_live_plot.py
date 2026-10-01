"""Tests for incremental live-plot telemetry reconstruction."""

import numpy as np

from fly_smart.simulation.live_plot import LIVE_FIELDS, PHASES, _append_rows
from fly_smart.simulation.telemetry import FlightLog


def test_append_rows_reconstructs_only_chart_fields_and_phase():
    log = FlightLog()
    rows = np.arange(2 * (len(LIVE_FIELDS) + 1), dtype=np.float64).reshape(2, -1)
    rows[:, -1] = (0, 2)

    _append_rows(log, rows)

    for column, field in enumerate(LIVE_FIELDS):
        assert getattr(log, field) == rows[:, column].tolist()
    assert log.phase == [PHASES[0], PHASES[2]]
    assert log.y_m == []

