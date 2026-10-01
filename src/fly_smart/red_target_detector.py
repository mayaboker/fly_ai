"""Detect the red camera target with HSV thresholding and draw its bounding box."""

import cv2
import numpy as np


def detect_red_box(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int] | None]:
    """Return an annotated BGR frame and the largest red target bounding box."""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    high_red = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
    mask = cv2.morphologyEx(low_red | high_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return bgr, None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 80:
        return bgr, None
    x, y, width, height = cv2.boundingRect(contour)
    cv2.rectangle(bgr, (x, y), (x + width, y + height), (0, 255, 255), 2)
    cv2.putText(bgr, f"red target: {width} x {height} px", (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return bgr, (x, y, width, height)
