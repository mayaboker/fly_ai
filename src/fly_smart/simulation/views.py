"""Forward-camera annotation and fixed environment-camera rendering."""

from math import degrees

import cv2
import numpy as np
import pybullet as p

from .config import StrikeConfig
from ..guidance import GuidanceCommand
from ..ttc import TtcObservation


def annotate(frame: np.ndarray, command: GuidanceCommand, observation: TtcObservation | None) -> np.ndarray:
    lines = [f"phase: {command.phase.value}", f"pitch: {degrees(command.pitch_target_rad):.1f} deg", f"thrust: {command.thrust_n:.2f} N"]
    if observation:
        lines.extend((f"scale: {observation.scale_px:.1f} px", f"growth: {observation.scale_growth_px_s:.1f} px/s", f"TTC: {observation.ttc_s:.2f} s"))
    if command.trajectory:
        lines.extend((f"vx command: {command.trajectory.forward_velocity_mps:.1f} m/s", f"vz command: {command.trajectory.vertical_velocity_mps:.1f} m/s"))
    for index, text in enumerate(lines):
        cv2.putText(frame, text, (12, 28 + 24 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def environment_rgb(renderer: int, config: StrikeConfig) -> np.ndarray:
    """Render a fixed wide view containing the launch point, path, and cube."""
    view = p.computeViewMatrix(cameraEyePosition=(7.0, -32.0, 22.0), cameraTargetPosition=(7.0, 0.0, 7.0), cameraUpVector=(0.0, 0.0, 1.0))
    width, height = config.environment_size_px
    projection = p.computeProjectionMatrixFOV(55.0, width / height, 0.1, 80.0)
    image = p.getCameraImage(width, height, view, projection, renderer=renderer)
    return np.reshape(image[2], (height, width, 4))[:, :, :3]
