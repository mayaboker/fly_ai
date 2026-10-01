from fly_smart.guidance import FlightPhase, GuidanceCommand
from fly_smart.simulation.telemetry import FlightLog
from fly_smart.trajectory import TrajectoryCommand


def test_telemetry_records_the_pid_vertical_target():
    command = GuidanceCommand(
        FlightPhase.TRACK,
        1.0,
        0.0,
        TrajectoryCommand(13.0, -1.5, 15.0),
        vertical_velocity_target_mps=-4.5,
    )
    log = FlightLog()
    log.append(0.0, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0), command)
    assert log.command_vz_mps == [-1.5]
    assert log.pid_vz_target_mps == [-4.5]
