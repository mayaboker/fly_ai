# Godot telemetry elapsed time and RTF

## Intent

Add attempt-relative wall time and real-time factor to the Godot telemetry
panel. This lets an operator compare simulated progress with actual runtime
without displaying the host's time of day. The values are display-only and do
not affect simulation time, physics, detection, TTC, or guidance.

## Design

Python starts a monotonic timer when the first physics step of an attempt
begins and resets it with the attempt. Camera-rate telemetry includes elapsed
wall seconds and real-time factor (`simulated time / elapsed wall time`). Godot
formats them as `ELAPSED MM:SS.s   RTF 0.00x` at the top of the existing HUD.
Before the attempt begins or after reset, unavailable values render as `--`.

## Validation

Load the project headlessly to catch script errors, run the Python tests and
self-check, and visually confirm that elapsed time and RTF update in the
telemetry panel without overlapping the flight values.
