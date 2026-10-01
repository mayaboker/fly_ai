# Simulation performance instrumentation

## Intent

Measure the complete Godot-backed simulation before changing its pacing or
visual quality. The instrumentation must identify which loop stages consume
the wall-clock budget and quantify whether the simulation sustains real time.
It must not alter physics, control, TTC, camera, or guidance behavior.

## Outputs and interface

The optional `--profile-performance` flag writes four artifacts beside the
normal run outputs: per-step `performance.csv`, one-second
`performance-counters.csv`, a Chrome/Perfetto `performance-trace.json`, and an
aggregate `performance-summary.json`. The terminal prints interval counters
and a ranked final summary. Real-time factor is simulated elapsed time divided
by active wall time.

Python records the major stages of every physics iteration and tracks camera
frame sequence continuity. Godot publishes renderer and shared-memory capture
counters over the existing event UDP connection once per second. Collision
events retain their existing semantics.

VS Code provides a non-interactive profiling task and a compound task that
starts Godot first. Profiling is disabled by default and no pacing or quality
optimization is part of this change.

## Validation

Unit tests cover aggregation, artifact schemas, camera sequence accounting,
and mixed performance/collision events. Run the Python suite, self-check,
Godot structural validation, and one complete profiled Godot mission.
