"""Show the RGB image from a forward-facing camera on the hovering drone."""

import argparse
from math import radians, tan
from pathlib import Path
import time

import cv2
import numpy as np
import pybullet as p

from .drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from .drone_physics import PhysicsEngine, clamp
from ..common.flight_control import AttitudeController
from .pybullet_sensors import read_imu, read_state
from .pybullet_utils import create_world, draw_force_vectors
from ..common.pid import PID

TARGET_ALTITUDE = 3.0
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
CAMERA_HZ = 30
MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
MASS = MODEL.mass_kg
PHYSICS_HZ = SETTINGS.physics_hz
TIME_STEP = SETTINGS.time_step_s
CONTROL_STEPS = SETTINGS.control_steps


def add_red_cube(center: tuple[float, float, float] = (20, 0, 1), size_m: float = 2.0, collision: bool = True) -> int:
    """Add a static red cube, optionally without a PyBullet collision shape."""
    half_extent = size_m / 2
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=(half_extent, half_extent, half_extent)) if collision else -1
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=(half_extent, half_extent, half_extent), rgbaColor=(0.9, 0.05, 0.05, 1))
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=center)


def add_environment_buildings() -> None:
    """Add distant static buildings for context without blocking the flight path."""
    for position, half_extents, color in (
        ((8, -9, 4), (3, 2, 4), (0.15, 0.35, 0.7, 1)),
        ((17, 8, 6), (2, 2, 6), (0.15, 0.5, 0.7, 1)),
        ((28, -7, 3), (3, 2, 3), (0.2, 0.4, 0.55, 1)),
    ):
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visual, basePosition=position)


def forward_rgb(
    drone: int,
    renderer: int,
    look_down_degrees: float = 0.0,
    mount_forward_m: float = 0.18,
    world_target: tuple[float, float, float] | None = None,
    width_px: int = CAMERA_WIDTH,
    height_px: int = CAMERA_HEIGHT,
    fov_deg: float = 60.0,
) -> np.ndarray:
    """Render a body camera, optionally with a stabilized optical axis."""
    position, orientation = p.getBasePositionAndOrientation(drone)
    eye, _ = p.multiplyTransforms(position, orientation, (mount_forward_m, 0.0, 0.03), (0, 0, 0, 1))
    target, _ = p.multiplyTransforms(position, orientation, (20.0, 0.0, 0.03 - 20.0 * tan(radians(look_down_degrees))), (0, 0, 0, 1))
    up_point, _ = p.multiplyTransforms(position, orientation, (0.0, 0.0, 1.0), (0, 0, 0, 1))
    up = tuple(axis - origin for axis, origin in zip(up_point, eye))
    view = p.computeViewMatrix(eye, world_target or target, (0, 0, 1) if world_target else up)
    projection = p.computeProjectionMatrixFOV(fov=fov_deg, aspect=width_px / height_px, nearVal=0.05, farVal=100.0)
    _, _, rgba, _, _ = p.getCameraImage(width_px, height_px, view, projection, renderer=renderer)
    # PyBullet's Python return type varies by build. Normalize signed integer
    # lists/arrays at the adapter boundary so OpenCV always receives RGB8.
    pixels = np.asarray(rgba, dtype=np.uint8)
    return pixels.reshape(height_px, width_px, 4)[:, :, :3]


def run(gui: bool, max_seconds: float) -> None:
    drone = create_world()
    engine = PhysicsEngine()
    add_red_cube()
    add_environment_buildings()
    altitude_pid = PID(kp=0.7, ki=0.05, kd=1.1, integral_limit=0.4)
    attitude_controller = AttitudeController()
    pwm = 1000.0
    torque = (0.0, 0.0, 0.0)
    force_lines = [-1, -1, -1, -1]
    renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER

    for step in range(round(max_seconds / TIME_STEP)):
        state = read_state(drone)
        position = state.position_m
        vertical_velocity = state.linear_velocity_mps[2]
        if step % CONTROL_STEPS == 0:
            total_thrust = MASS * 9.81 + altitude_pid.update(TARGET_ALTITUDE - position[2], vertical_velocity)
            pwm = engine.pwm_from_thrust(clamp(total_thrust / 4, 0.0, MODEL.max_thrust_per_motor_n))
            torque = attitude_controller.update(read_imu(drone), yaw_target=0.0)
        flight_step = engine.step(drone, pwm, torque)

        if step % (PHYSICS_HZ // CAMERA_HZ) == 0:
            rgb = forward_rgb(drone, renderer)
            if gui:
                cv2.imshow("Drone forward RGB camera", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    return

        if gui:
            draw_force_vectors(drone, flight_step, force_lines)
            time.sleep(TIME_STEP)


def self_check() -> None:
    drone = create_world()
    add_red_cube()
    add_environment_buildings()
    image = forward_rgb(drone, p.ER_TINY_RENDERER)
    red_pixels = (image[:, :, 0] > 120) & (image[:, :, 0] > image[:, :, 1] * 2) & (image[:, :, 0] > image[:, :, 2] * 2)
    assert image.shape == (CAMERA_HEIGHT, CAMERA_WIDTH, 3), "Forward camera must return an RGB image"
    assert image.dtype == np.uint8, "Forward camera must normalize PyBullet pixels to RGB8"
    assert red_pixels.any(), "Forward camera should see the red target cube"
    print("Forward camera self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            run(not args.headless, args.max_seconds)
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
