import numpy as np
from dataclasses import replace
from math import degrees

from fly_smart.guidance import FlightPhase, GuidanceInput, StrikeGuidance
from fly_smart.mission import MissionConfig
from fly_smart.red_target_detector import detect_red_box
from fly_smart.sensing import BarometerReading
from fly_smart.ttc import BboxTtcTracker
from fly_smart.trajectory import TtcDescentPlanner


def test_core_guidance_and_vision_do_not_need_simulation():
    config = MissionConfig()
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation is not None and observation.ttc_s > 0
    guidance = StrikeGuidance(config)
    command = guidance.update(GuidanceInput(0.0, BarometerReading(0.0, 0.0), None, None, False, False))
    assert command.thrust_n > config.hover_thrust_n

    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[20:70, 20:70] = (255, 0, 0)
    _, box = detect_red_box(frame)
    assert box == (20, 20, 50, 50)

    frame[:, :] = (20, 89, 235)
    _, box = detect_red_box(frame)
    assert box is None


def test_ttc_unavailable_descent_feedforward_is_bounded_and_optional():
    config = MissionConfig()
    planner = TtcDescentPlanner(config)
    assert planner.command(None, config.takeoff_altitude_m).vertical_velocity_mps == -1.5
    assert TtcDescentPlanner(replace(config, ttc_unavailable_descent_velocity_mps=0.0)).command(None, config.takeoff_altitude_m).vertical_velocity_mps == 0.0
    assert planner.command(1.0, config.takeoff_altitude_m).vertical_velocity_mps == -config.max_descent_velocity_mps


def test_pitch_boost_latches_until_ttc_decreases_reliably():
    config = replace(MissionConfig(), ttc_unavailable_pitch_boost_deg=5.0)
    tracker = BboxTtcTracker(config)
    tracker.update((0, 0, 20, 20), 0.0)
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation is not None
    guidance = StrikeGuidance(config)
    reading = BarometerReading(config.takeoff_altitude_m, 0.0)
    guidance.update(GuidanceInput(0.0, reading, observation, observation, True, False))
    assert guidance.phase == FlightPhase.TRACK

    boosted = guidance.update(GuidanceInput(0.1, reading, None, observation, True, False))
    baseline = guidance.update(GuidanceInput(0.2, reading, observation, observation, True, False))
    decreasing = replace(observation, ttc_s=observation.ttc_s * 0.8)
    guidance.update(GuidanceInput(0.3, reading, decreasing, decreasing, True, False))
    guidance.update(GuidanceInput(0.4, reading, replace(decreasing), decreasing, True, False))
    released = guidance.update(GuidanceInput(0.5, reading, replace(decreasing), decreasing, True, False))
    assert degrees(boosted.pitch_target_rad) == 25.0
    assert degrees(baseline.pitch_target_rad) == 25.0
    assert round(degrees(released.pitch_target_rad), 6) == 23.0

    unboosted = StrikeGuidance(MissionConfig())
    unboosted.update(GuidanceInput(0.0, reading, observation, observation, True, False))
    unavailable = unboosted.update(GuidanceInput(0.1, reading, None, observation, True, False))
    assert degrees(unavailable.pitch_target_rad) == 20.0
