# Godot FPV HUD

## Intent

Use the existing Godot window as the only live camera display. Keep the
spectator view and draw flight telemetry plus Python's detected target bounding
box over the FPV inset. Remove the separate OpenCV preview window and its CLI
option.

Python remains authoritative for physics, target detection, TTC, and guidance.
It publishes display-only telemetry over the existing pose UDP connection.
Godot formats and draws that data but never feeds it back into control.

## Data flow and display

Pose packets on UDP port 9100 gain an optional `telemetry` object. It carries
simulation time, phase, position, velocity, filtered vertical state, measured
and commanded pitch, thrust, target visibility, bbox and TTC measurements, and
trajectory velocity commands. Unavailable or non-finite measurements are JSON
`null`.

The bbox uses coordinates from the 640 x 360 shared-memory image. Godot scales
it to the 480 x 270 FPV preview. The HUD is attached to the root CanvasLayer,
not the FPV SubViewport, so shared-memory camera images stay free of overlays
and the detector cannot observe its own annotations. Reset packets clear all
display state.

## Validation

Unit tests cover telemetry sanitization and optional telemetry packets. Run the
Python tests and self-check, validate that Godot parses the project, then run a
Godot-backed mission and confirm the bbox aligns with the target, disappears
when detection is lost, and no OpenCV camera window opens.
