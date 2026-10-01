# Early TTC Descent Feed-Forward

## Problem

Tracking starts after takeoff, but the bounding-box TTC estimate is initially
large or unavailable. The existing planner commands zero vertical velocity in
that interval. A vehicle that overshoots takeoff altitude must first shed that
extra height after TTC becomes useful, leaving too little distance to intercept
the target.

## Decision

While tracking without a valid TTC measurement, command a bounded vertical
target of `-1.5 m/s`. Keep the takeoff altitude as the trajectory hold
reference, so the existing altitude correction and vertical-velocity PID remain
responsible for safe thrust control. Once TTC is valid, retain the current
TTC-synchronised trajectory unchanged.

`runtime.ttc.ttc_unavailable_descent_velocity_mps` stores the positive
magnitude. It defaults to `1.5`; `0` disables the feed-forward.

For the seven-inch trainer only, a visible target with no valid TTC receives a
`5 deg` pitch addition after normal forward-speed control. Its ordinary limit
remains `20 deg`; this fallback can command at most `25 deg`. It is disabled by
default with `runtime.ttc.ttc_unavailable_pitch_boost_deg: 0`. The addition is
not applied once TTC becomes valid or when the target is absent, so it cannot
extend terminal commit or target-loss behavior.

Enable the same `5 deg` addition in the maintained default scenario, allowing
an absolute pitch command of at most `25 deg` while TTC is unavailable. The
configuration field is an additive boost, not an absolute pitch limit;
configuring it as `25 deg` would incorrectly permit up to `45 deg`. This tuning
does not change the existing release condition: the boost ends when the first
valid TTC observation becomes available.

## Observability and validation

Telemetry records both the raw trajectory vertical velocity and the corrected
vertical target supplied to the PID. This prevents the plotted raw TTC target
from being mistaken for the actual PID demand.

Validate the planner with no TTC, disabled feed-forward, and valid TTC. Run
both maintained headless scenarios. The seven-inch scenario keeps
`vertical_position_correction: 2.0`, which independently maintains descent as
TTC grows near the target.

Validate the pitch fallback with visible/no-TTC, valid-TTC, and target-lost
inputs. In the seven-inch headless run, confirm commanded pitch remains at or
below `25 deg` and the strike succeeds.

## Implementation review

The planner, guidance fallback, configuration fields, corrected vertical
target telemetry, and focused tests are already present in the current code.
The remaining validation gap is in the simulation adapter: the installed
PyBullet binding returns camera pixels as a signed integer array, while OpenCV
5 accepts only supported image depths. Normalize PyBullet RGBA samples to
`uint8` inside `forward_rgb()` before detection. This is a representation-only
boundary fix and does not change TTC estimation or guidance behavior.

The default target's new 50 m horizontal separation also exceeds the
compatibility camera's 50 m far plane once the altitude offset is included.
Increase that renderer-only far plane to 100 m so the maintained headless
scenario observes the same target that Godot can see. Keep FOV, detector
thresholds, TTC filtering, and all flight commands unchanged.

Godot validation evidence: the seven-inch trainer began tracking at 5.200 s
with a raw `-1.5 m/s` vertical command and a 25 deg pitch command. TTC became
valid at 6.167 s, where pitch returned to 20 deg. Maximum commanded pitch was
25 deg, and target contact occurred at 12.508 s with an 8.456 m/s impact.

Headless validation evidence after camera normalization: the default scenario
contacted at 13.200 s with a 7.300 m/s impact, and the seven-inch scenario
contacted at 12.567 s with an 8.378 m/s impact. The default scenario's prior
4.396 s target-loss abort was reproduced with the 50 m far plane and resolved
by the 100 m renderer-only far plane. All focused tests and the self-check pass.

## Smooth diagonal acquisition revision

The latest default run exposed command discontinuities rather than a vehicle
dynamics problem. A missing TTC sample toggled pitch between boosted and normal
limits, and the 15 m to 1 m altitude-reference transition drove the corrected
vertical target into its `-4.5 m/s` limit. The vehicle consequently descended
steeply and then levelled near the target instead of following one diagonal.

Latch acquisition boost on entry to TRACK. Record the first valid filtered TTC
as a baseline and release the boost only after three consecutive valid samples
are at least 10 percent below that baseline. Preserve the last valid TTC across
brief visible-target samples where growth is temporarily invalid. Clear the
latch and cached TTC on every new attempt or TRACK transition.

Rate-limit the pitch target to 20 deg/s and the corrected vertical target to
2 m/s² using control timestamps. This applies after bounding the ordinary
commands and prevents estimator noise from reaching the attitude and thrust
loops as steps. Reduce the default scenario's
`vertical_position_correction` from `0.8` to `0.3`. TTC already recomputes the
required vertical rate from current altitude, so the former correction
double-counted that error and saturated descent; zero correction, however, did
not overcome the vehicle's initial vertical response lag, and `0.2` crossed
just above the truck at 2.7 m. The small `0.3` correction supplies convergence
authority while the slew limit prevents a step. The
seven-inch scenario keeps its independently tuned correction until separately
reviewed.

Validation must cover acquisition latching, noisy/non-decreasing TTC, three
qualifying decreasing samples, missing samples after a valid TTC, and both
slew limits. A full default run must remain successful, never exceed 25 deg,
avoid alternating boost spikes, and show a progressive diagonal descent with
no `-4.5 m/s` corrected-target saturation.

Revision validation: zero correction produced a smooth but shallow pass at
about 6 m, and `0.2` crossed just above the truck at 2.7 m. With `0.3`, the
Godot mission contacted the truck at 13.058 s and 2.247 m altitude with a
7.310 m/s impact. During TRACK, the largest per-control-tick pitch change was
0.167 deg and the largest corrected vertical-target change was 0.017 m/s. The
vehicle held 25 deg during acquisition, then followed a continuous descent
from 15.2 m to contact without the former late level segment or command
flicker.
