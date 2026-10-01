"""Ideal sensor and frame-transform helpers backed by PyBullet state."""

import pybullet as p

from .drone_model import DroneState, ImuReading


def world_to_body_vector(drone: int, world_vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Transform a world-frame vector into the drone's rotating body frame."""
    rotation = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    return (
        rotation[0] * world_vector[0] + rotation[3] * world_vector[1] + rotation[6] * world_vector[2],
        rotation[1] * world_vector[0] + rotation[4] * world_vector[1] + rotation[7] * world_vector[2],
        rotation[2] * world_vector[0] + rotation[5] * world_vector[1] + rotation[8] * world_vector[2],
    )


def body_to_world_vector(drone: int, body_vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Transform a body-frame vector into the fixed world frame."""
    rotation = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    return (
        rotation[0] * body_vector[0] + rotation[1] * body_vector[1] + rotation[2] * body_vector[2],
        rotation[3] * body_vector[0] + rotation[4] * body_vector[1] + rotation[5] * body_vector[2],
        rotation[6] * body_vector[0] + rotation[7] * body_vector[1] + rotation[8] * body_vector[2],
    )


def read_state(drone: int) -> DroneState:
    """Return the complete ideal rigid-body state from PyBullet."""
    position, orientation = p.getBasePositionAndOrientation(drone)
    linear_velocity, angular_velocity_world = p.getBaseVelocity(drone)
    return DroneState(
        tuple(position),
        tuple(linear_velocity),
        tuple(orientation),
        world_to_body_vector(drone, tuple(angular_velocity_world)),
    )


def read_imu(drone: int) -> ImuReading:
    """Return ideal Euler attitude and body-frame angular rate as an IMU would."""
    state = read_state(drone)
    return ImuReading(p.getEulerFromQuaternion(state.orientation_quaternion), state.angular_velocity_body_rad_s)
