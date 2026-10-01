# Godot 30 Hz FPV capture

## Intent

Align Godot's unique shared-memory FPV production with the configured 30 Hz
Python camera cadence. Preserve the 640 x 360 Godot buffer, Terrain3D quality,
physics rate, control rate, UDP ports, and shared-memory binary format.

## Capture algorithm

Python publishes the scenario `camera_hz` as renderer settings at startup and
reset. Godot defaults to 30 Hz and validates the received rate. After each
rendered frame, an absolute monotonic deadline decides whether one image is
due. The deadline advances by one capture period; the renderer never queues
multiple stale captures, and lateness greater than one period rebases the
schedule. This removes the old behavior that added a full timer interval after
the synchronous image readback.

Performance telemetry reports target and measured capture rates, deadline
misses, rebases, readback duration, and shared-memory write duration. If the
machine cannot sustain at least 28 unique frames per second, retain image and
terrain quality and report the measured ceiling.

## TTC and guidance impact

The Python loop already invokes detection and `BboxTtcTracker.update()` at a
nominal 30 Hz with simulation timestamps. The prior 13--14 Hz producer caused
many duplicate images to be treated as new measurements. A true 30 Hz stream
provides more distinct bbox scales and can therefore change raw scale growth,
the alpha-beta residual sequence, TTC estimates, commit timing, collision
time, and impact state. This change does not alter `ttc_alpha`, `ttc_beta`, the
commit threshold, trajectory logic, or guidance state machine.

Validation must compare bbox scale/growth, TTC, phase transition, collision
time, collision position, and impact speed with the committed baseline. Any
follow-up tuning requires separate evidence and design review.

## Validation

Test renderer-settings serialization and invalid-rate fallback, validate the
Godot project, and run profiled missions with video and asynchronous plotting.
Over a steady ten-second interval, expect 28--32 unique frames per second,
approximately 30 Python polls per second, Godot rendering at or above 30 FPS,
RTF from 0.98 to 1.02, and a successful target collision.

Implementation evidence on the reference machine: the video-enabled mission
produced 379 unique frames in 13.56 s (27.95 Hz including startup; 28.70 Hz
mean from seconds 3--13), while the plot-enabled mission produced 392 unique
frames in 13.54 s (28.95 Hz) with 15 duplicate polls. Both held RTF 1.000 and
contacted the target. Against the committed pacing baseline, collision time
moved from 10.575 s to 10.558 s, impact speed from 7.440 m/s to 7.456 m/s,
track entry remained 4.017 s, and last pre-impact TTC moved from 0.970 s to
0.990 s. No TTC or guidance retuning is indicated.
