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
