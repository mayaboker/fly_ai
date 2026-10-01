# Deadline-based real-time pacing

## Intent

Make GUI and Godot simulations advance at real-time pace without adding a full
physics timestep after every iteration. Preserve every physics, control,
camera, TTC, and telemetry update. Pure headless runs remain unpaced.

## Design

A monotonic pacer advances an absolute deadline by one physics timestep after
each completed step. It sleeps only for the remaining budget. Short overruns
skip sleep and recover on later inexpensive steps. Lag greater than 250 ms
rebases the deadline to avoid an extended unthrottled catch-up burst.

Restart and interactive resume establish a fresh pacing deadline. Performance
artifacts record requested and actual sleep, real deadline lateness, and rebase
counts. The HUD computes RTF from completed simulated time rather than the
start timestamp of the current step.

## Validation

Use a fake clock to test remaining-budget sleep, overrun recovery, long-stall
rebasing, and reset behavior. Run matched profiled Godot missions with video,
asynchronous plotting, and interactive controls. A normal run should finish
between 0.98 and 1.02 RTF without skipping simulation steps.
