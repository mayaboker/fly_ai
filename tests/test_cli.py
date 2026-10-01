from dataclasses import replace

import argparse
import pytest

from fly_smart.simulation.cli import target_distance, with_target_distance
from fly_smart.simulation.config import SimulationConfig, StrikeConfig


def test_default_target_is_thirty_metres_ahead_of_launch():
    config = StrikeConfig()

    assert config.simulation.target_center == (24.75, 0.0, 1.0)
    assert config.simulation.target_center[0] - config.simulation.launch_position[0] == 30.0


def test_target_distance_override_is_relative_and_preserves_target_yz():
    simulation = replace(
        SimulationConfig(),
        launch_position=(10.0, 3.0, 0.05),
        target_center=(99.0, -2.0, 4.0),
    )

    resolved = with_target_distance(StrikeConfig(simulation=simulation), 50.0)

    assert resolved.simulation.target_center == (60.0, -2.0, 4.0)
    assert resolved.simulation.launch_position == simulation.launch_position


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "-inf", "not-a-number"])
def test_target_distance_rejects_invalid_values(value: str):
    with pytest.raises(argparse.ArgumentTypeError):
        target_distance(value)


def test_target_distance_accepts_positive_finite_values():
    assert target_distance("30") == 30.0
