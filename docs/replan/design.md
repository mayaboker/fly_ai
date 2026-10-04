# ADRC-Based Drone Visual Tracking System
## Simulation Design Document

## 1. Purpose

This document defines the architecture of a simulated drone visual-tracking system based on ADRC.

The system should:

1. Detect and track a target.
2. Center the target in the camera.
3. Approach the target.
4. Use target scale and TTC during approach.
5. Enter a terminal-alignment phase.
6. Enter a simulation-only COMMIT phase when predefined conditions are satisfied.

The architecture separates:

- perception,
- measurement validation,
- state estimation,
- guidance,
- mode management,
- ADRC control,
- command allocation,
- autopilot stabilization,
- logging and simulation instrumentation.

---

# 2. High-Level Architecture

```text
Camera
  ↓
Detector / Tracker
  ↓
Measurement Validation
  ↓
Measurement Conversion
  ↓
Target State Estimator
  ↓
Latency Compensation
  ↓
LOS + Scale + TTC Estimation
  ↓
Guidance Layer
  ↓
Mode Manager
  ↓
Mode Blending
  ↓
ADRC Controllers
  ↓
Command Allocation
  ↓
Command Limits / Safety
  ↓
Autopilot Inner Loops
  ↓
Simulated UAV
  ↓
Camera Feedback
```

---

# 3. Detector Output

The detector/tracker provides:

```text
u
v
bbox_width
bbox_height
confidence
timestamp
```

Where:

- `u, v` = target bounding-box center
- `bbox_width` = bounding-box width in pixels
- `bbox_height` = bounding-box height in pixels
- `confidence` = detector/tracker confidence
- `timestamp` = image measurement time

Camera parameters:

```text
image_width  = W
image_height = H

principal point = (cx, cy)

focal lengths:
fx
fy
```

---

# 4. Image Measurement Conversion

## 4.1 Pixel error

```text
ex = u - cx
ey = v - cy
```

Normalized errors:

```text
ex_norm = (u - cx) / (W / 2)
ey_norm = (v - cy) / (H / 2)
```

---

## 4.2 Line-of-Sight Angles

Prefer angular LOS measurements over raw pixels.

```text
theta_x = atan((u - cx) / fx)

theta_y = atan((v - cy) / fy)
```

Normal tracking objective:

```text
theta_x → 0
theta_y → 0
```

---

# 5. Target Scale

Use apparent bounding-box size as a monocular range proxy.

Primary recommended scale:

```text
scale_h = bbox_height / image_height
```

or:

```text
s = h_bbox / H
```

Additional optional scale measurements:

```text
scale_w = bbox_width / image_width

scale_area =
    (bbox_width * bbox_height)
    /
    (image_width * image_height)
```

For the first implementation, use:

```text
s = bbox_height / image_height
```

because bounding-box height is easier to interpret and less sensitive than area.

---

# 6. Measurement Validation

Raw detector output should not go directly to the estimator.

```text
Detector
   ↓
Measurement Validation
   ├── confidence check
   ├── timestamp check
   ├── bbox sanity check
   ├── bbox jump rejection
   ├── clipping detection
   └── stale frame detection
   ↓
Estimator
```

Important validity checks:

```text
confidence > confidence_min

timestamp is newer than previous measurement

bbox_width > 0
bbox_height > 0
```

Reject physically unreasonable jumps in:

```text
u
v
bbox_width
bbox_height
```

---

# 7. Target Clipping

The system should know if the target bounding box reaches the camera boundary.

Example:

```text
target_clipped =
    bbox_left   <= margin
    OR
    bbox_right  >= W - margin
    OR
    bbox_top    <= margin
    OR
    bbox_bottom >= H - margin
```

When:

```text
target_clipped = true
```

the scale estimate becomes less trustworthy.

---

# 8. State Estimator

The estimated visual state is:

```text
theta_x
theta_x_dot

theta_y
theta_y_dot

scale
scale_dot
```

Conceptually:

```text
state =
[
    theta_x,
    theta_x_dot,
    theta_y,
    theta_y_dot,
    scale,
    scale_dot
]
```

The estimator should also output:

```text
LOS_quality
scale_quality

measurement_age
prediction_age
```

---

# 9. Recommended Initial Filter

Use three independent alpha-beta filters.

```text
Alpha-Beta X
    theta_x
    theta_x_dot

Alpha-Beta Y
    theta_y
    theta_y_dot

Alpha-Beta Scale
    scale
    scale_dot
```

Advantages:

- simple,
- computationally cheap,
- easy to tune,
- gives both position and rate,
- works well as a first constant-velocity estimator.

A Kalman filter can be introduced later if IMU, optical flow, range sensors, or explicit uncertainty fusion are added.

---

# 10. Alpha-Beta Filter

For a generic state:

```text
x     = value
v     = rate
z     = measurement
dt    = elapsed time
```

Prediction:

```text
x_pred = x_prev + dt * v_prev
```

Residual:

```text
r = z - x_pred
```

Update:

```text
x_new = x_pred + alpha * r
```

Rate update:

```text
v_new = v_prev + (beta / dt) * r
```

Apply independently to:

```text
theta_x
theta_y
scale
```

---

# 11. Scale Filtering

Do not directly calculate:

```text
scale_dot =
    (scale_now - scale_previous)
    /
    dt
```

from raw bounding boxes.

Bounding-box jitter will strongly affect the derivative.

Use:

```text
bbox
 ↓
scale measurement
 ↓
alpha-beta filter
 ↓
scale_hat
scale_dot_hat
 ↓
TTC estimator
```

---

# 12. Latency Compensation

The detector measurement is delayed relative to the current UAV state.

Define:

```text
latency =
    current_time
    -
    measurement_timestamp
```

The estimator should predict the state forward to the current control time.

For example:

```text
theta_x_now =
    theta_x_hat
    +
    theta_x_dot_hat * latency
```

Similarly:

```text
theta_y_now =
    theta_y_hat
    +
    theta_y_dot_hat * latency
```

and:

```text
scale_now =
    scale_hat
    +
    scale_dot_hat * latency
```

This becomes increasingly important during fast terminal motion.

---

# 13. TTC Estimate

Using filtered scale:

```text
s = scale_hat
```

and filtered scale rate:

```text
s_dot = scale_dot_hat
```

a TTC-like value is:

```text
TTC ≈ s / s_dot
```

This should only be calculated when `s_dot` is sufficiently large and has the correct sign.

---

# 14. TTC Validity

If:

```text
abs(scale_dot_hat) < scale_dot_min
```

then:

```text
TTC_valid = false
```

Additional TTC validity conditions:

```text
scale_quality is good

target_clipped == false

measurement_age < maximum_age

confidence > confidence_min

scale_dot has been stable for several frames
```

Recommended output:

```text
TTC
TTC_valid
TTC_quality
```

---

# 15. Why TTC Is Not Enough

When the target is far away:

```text
scale is small
scale_dot ≈ 0
```

TTC becomes very large or unstable.

Therefore the longitudinal guidance should use:

```text
scale
+
scale rate
+
TTC validity
```

rather than TTC alone.

---

# 16. Meaning of the Range Signals

```text
scale
```

answers:

> How large does the target currently appear?

```text
scale_dot
```

answers:

> How quickly is the apparent target size changing?

```text
TTC
```

answers:

> How rapidly is the current closing geometry evolving?

---

# 17. LOS Guidance

Desired LOS references:

```text
theta_x_ref = 0
theta_y_ref = 0
```

The objective is to keep the target near the optical center.

---

# 18. ADRC Controllers

Use three logical ADRC channels:

```text
ADRC-X
ADRC-Y
ADRC-RANGE
```

---

## 18.1 ADRC-X

Purpose:

```text
horizontal target alignment
```

Error:

```text
error_x =
    theta_x_ref
    -
    theta_x_hat
```

Recommended output:

```text
yaw_rate_request
```

---

## 18.2 ADRC-Y

Purpose:

```text
vertical target alignment
```

Error:

```text
error_y =
    theta_y_ref
    -
    theta_y_hat
```

Preferred output:

```text
vertical_velocity_request
```

An initial simpler design could output pitch directly, but this causes coupling with longitudinal motion.

---

## 18.3 ADRC-RANGE

Purpose:

```text
longitudinal approach control
```

Inputs may include:

```text
scale
scale_dot
TTC
TTC_valid
current_mode
```

Recommended output:

```text
forward_velocity_request
```

---

# 19. Visual Estimator vs ADRC ESO

These should remain separate.

The visual estimator handles:

```text
camera noise
detector jitter
target motion
missing measurements
latency
scale-rate estimation
```

The ADRC Extended State Observer handles:

```text
vehicle dynamics
plant uncertainty
external disturbances
controller-model mismatch
coupling
```

Therefore:

```text
Visual State Estimator
        +
ADRC ESO
```

are complementary.

---

# 20. Mode Manager

Main mode sequence:

```text
SEARCH
   ↓
TRACK
   ↓
FAR APPROACH
   ↓
TTC APPROACH
   ↓
TERMINAL ALIGN
   ↓
COMMIT
   ↓
INTERCEPT EVENT
```

Additional modes:

```text
LOST TARGET
RECOVERY
SAFETY / ABORT
```

Optional test mode:

```text
RANGE HOLD
```

---

# 21. SEARCH

Purpose:

```text
acquire a valid target
```

Transition:

```text
SEARCH
   ↓ target detected consistently
TRACK
```

---

# 22. TRACK

Purpose:

```text
center the target
```

Objectives:

```text
theta_x → 0
theta_y → 0
```

Transition to FAR APPROACH when:

```text
target stable
AND
target approximately centered
AND
confidence acceptable
```

---

# 23. FAR APPROACH

Used when:

```text
target is far
scale is small
scale_dot ≈ 0
TTC invalid
```

Behavior:

```text
continue LOS tracking
+
bounded forward approach
```

Conceptually:

```text
if target_centered
and target_valid
and TTC_valid == false:
    command slow forward velocity
```

---

# 24. TTC APPROACH

Enter when:

```text
scale_dot becomes measurable
AND
TTC becomes valid
```

Behavior:

```text
maintain LOS alignment

monitor scale growth

monitor TTC

adjust forward approach
```

---

# 25. Optional RANGE HOLD

Useful during development.

Objective:

```text
scale → desired_scale
```

or:

```text
scale_error =
    desired_scale
    -
    scale_hat
```

This allows the scale controller to be tested before terminal modes are enabled.

---

# 26. TERMINAL ALIGN

Enter before COMMIT.

Purpose:

```text
reduce LOS error

reduce LOS rate

ensure stable tracking

verify scale estimate

prepare for COMMIT
```

Example transition into terminal alignment:

```text
scale > 0.45
```

This value is only an initial simulation parameter.

---

# 27. COMMIT Scale Trigger

Initial proposal:

```text
scale =
    bbox_height
    /
    image_height
```

COMMIT scale condition:

```text
scale > 0.60
```

However, this must not be the only condition.

---

# 28. Recommended COMMIT Gate

```text
COMMIT_ALLOWED =
    scale_large
    AND horizontal_alignment_good
    AND vertical_alignment_good
    AND horizontal_LOS_rate_good
    AND vertical_LOS_rate_good
    AND tracker_stable
    AND target_not_clipped
    AND measurements_valid
```

---

# 29. Example COMMIT Conditions

Scale:

```text
scale > 0.60
```

Horizontal LOS:

```text
abs(theta_x)
<
theta_x_commit_max
```

Vertical LOS:

```text
abs(theta_y)
<
theta_y_commit_max
```

Horizontal LOS rate:

```text
abs(theta_x_dot)
<
theta_x_dot_commit_max
```

Vertical LOS rate:

```text
abs(theta_y_dot)
<
theta_y_dot_commit_max
```

Tracker:

```text
confidence
>
confidence_commit_min
```

Measurement conditions:

```text
TTC_valid == true

target_clipped == false

measurement_age < max_measurement_age
```

---

# 30. Why LOS Rate Matters

A target can briefly pass through the image center while moving rapidly across the image.

Therefore this is insufficient:

```text
small LOS error
```

A better terminal condition is:

```text
small LOS error
+
small LOS rate
```

---

# 31. Persistence

Do not enter COMMIT because of a single frame.

Use:

```text
COMMIT conditions true
for N consecutive frames
```

or preferably:

```text
COMMIT conditions true
for T_commit seconds
```

Possible initial simulation range:

```text
T_commit = 0.1–0.2 s
```

This should remain tunable.

---

# 32. Hysteresis

Use different entry and cancellation thresholds.

Example:

```text
ENTER COMMIT candidate:
    scale > 0.60

CANCEL COMMIT candidate:
    scale < 0.50
```

This avoids rapid switching caused by measurement noise.

---

# 33. COMMIT Phase

Before COMMIT, range control may use:

```text
scale → desired_scale
```

During COMMIT, range-hold behavior is disabled.

The simulated terminal guidance priorities become:

```text
1. Maintain horizontal LOS alignment

2. Maintain vertical LOS alignment

3. Keep LOS rates small

4. Continue simulated closing motion
```

---

# 34. State Transition Diagram

```text
                         SEARCH
                           │
                    target acquired
                           ▼
                         TRACK
                           │
                   centered + stable
                           ▼
                     FAR APPROACH
                           │
                   scale_dot reliable
                           ▼
                     TTC APPROACH
                           │
                scale > terminal threshold
                           ▼
                   TERMINAL ALIGN
                           │
             scale > commit threshold
             LOS error acceptable
             LOS rate acceptable
             tracking stable
             not clipped
             persistence satisfied
                           ▼
                         COMMIT
                           │
                           ▼
                  INTERCEPT EVENT
```

---

# 35. Lost Target Handling

Possible loss conditions:

```text
confidence too low

bbox disappeared

frame stale

target exits FOV

prediction_age too large
```

Suggested behavior:

```text
tracking
   ↓
brief loss
   ↓
prediction-only period
   ↓
target recovered?
   ├── yes → resume
   └── no  → LOST TARGET
                  ↓
               RECOVERY
                  ↓
                SEARCH
```

---

# 36. Prediction-Only Grace Period

During a short measurement loss, use predicted values:

```text
theta_x_hat

theta_x_dot_hat

theta_y_hat

theta_y_dot_hat

scale_hat

scale_dot_hat
```

Monitor:

```text
prediction_age
```

If:

```text
prediction_age > max_prediction_age
```

then:

```text
estimator_valid = false
```

and leave active approach/terminal modes.

---

# 37. Mode Blending

Avoid abrupt transitions between:

```text
TRACK → FAR APPROACH

FAR APPROACH → TTC APPROACH

TTC APPROACH → TERMINAL ALIGN
```

Use command blending:

```text
command =
    (1 - lambda) * old_command
    +
    lambda * new_command
```

with:

```text
0 <= lambda <= 1
```

---

# 38. Command Allocation

The visual controller should output motion intent rather than motor commands.

Preferred controller outputs:

```text
yaw_rate_request

forward_velocity_request

vertical_velocity_request
```

The autopilot converts these into:

```text
roll
pitch
yaw
thrust
```

---

# 39. Why Not Use Pitch for Two Loops

If vertical tracking does:

```text
theta_y → pitch
```

while longitudinal range control also does:

```text
forward motion → pitch
```

two controllers compete for the same actuator.

Preferred architecture:

```text
ADRC-X
   ↓
yaw-rate reference


ADRC-Y
   ↓
vertical-velocity reference


ADRC-RANGE
   ↓
forward-velocity reference
```

Then:

```text
velocity references
       ↓
autopilot
       ↓
roll / pitch / yaw / thrust
```

---

# 40. Command Limits

Configure limits such as:

```text
max_yaw_rate

max_forward_velocity

max_vertical_velocity

max_acceleration

max_pitch

max_roll
```

Also apply:

```text
command slew-rate limits
```

to prevent abrupt control changes.

---

# 41. Safety / Validity Layer

```text
ADRC Outputs
    ↓
Command Allocation
    ↓
Safety / Limits
    ├── saturation
    ├── slew-rate limits
    ├── confidence gate
    ├── estimator validity
    ├── TTC validity
    ├── mode validity
    └── target-loss handling
    ↓
Autopilot
```

---

# 42. Autopilot Responsibilities

The visual controller should not directly command simulated motors.

The FCU handles:

```text
velocity control

attitude control

angular-rate control

altitude control

motor mixing
```

Architecture:

```text
Visual ADRC
    ↓
Motion References
    ↓
Autopilot
    ↓
Vehicle Dynamics
```

---

# 43. Complete Architecture

```text
CAMERA
  │
  ▼
DETECTOR / TRACKER
  │
  │ u, v, bbox_w, bbox_h
  │ confidence, timestamp
  ▼
MEASUREMENT VALIDATION
  │
  ├── confidence
  ├── clipping
  ├── jumps
  ├── stale data
  └── timestamps
  │
  ▼
MEASUREMENT CONVERSION
  │
  ├── theta_x
  ├── theta_y
  └── scale
  │
  ▼
STATE ESTIMATOR
  │
  ├── theta_x
  ├── theta_x_dot
  ├── theta_y
  ├── theta_y_dot
  ├── scale
  └── scale_dot
  │
  ▼
LATENCY COMPENSATION
  │
  ├───────────────┐
  │               │
  ▼               ▼
LOS GUIDANCE   SCALE / TTC
  │               │
  └───────┬───────┘
          ▼
      MODE MANAGER
          │
          ├── SEARCH
          ├── TRACK
          ├── FAR APPROACH
          ├── TTC APPROACH
          ├── RANGE HOLD
          ├── TERMINAL ALIGN
          ├── COMMIT
          ├── LOST / RECOVERY
          └── SAFETY / ABORT
          │
          ▼
      MODE BLENDING
          │
    ┌─────┼─────────┐
    │     │         │
    ▼     ▼         ▼
 ADRC-X ADRC-Y ADRC-RANGE
    │     │         │
    └─────┼─────────┘
          ▼
  COMMAND ALLOCATION
          │
          ▼
    LIMITS / SAFETY
          │
          ▼
       AUTOPILOT
          │
          ▼
    SIMULATED UAV
          │
          └──────────────► CAMERA
```

---

# 44. Main Software Interfaces

## PerceptionOutput

```text
timestamp
target_valid

center_x
center_y

bbox_width
bbox_height

confidence
```

## MeasurementState

```text
timestamp

theta_x_meas
theta_y_meas

scale_meas

target_clipped
measurement_valid
```

## EstimatedTargetState

```text
timestamp

theta_x
theta_x_dot

theta_y
theta_y_dot

scale
scale_dot

LOS_quality
scale_quality

measurement_age
prediction_age
```

## RangeState

```text
scale
scale_dot

TTC
TTC_valid
TTC_quality
```

## GuidanceState

```text
mode

theta_x_ref
theta_y_ref

forward_velocity_ref
vertical_velocity_ref
yaw_rate_ref
```

## ControllerOutput

```text
yaw_rate_cmd
forward_velocity_cmd
vertical_velocity_cmd
```

---

# 45. Logging

Log at minimum:

```text
timestamp
mode

raw_u
raw_v

raw_bbox_width
raw_bbox_height

confidence

theta_x_measured
theta_y_measured
scale_measured

theta_x_estimated
theta_x_dot_estimated

theta_y_estimated
theta_y_dot_estimated

scale_estimated
scale_dot_estimated

TTC
TTC_valid

yaw_rate_reference
forward_velocity_reference
vertical_velocity_reference

ADRC_X_output
ADRC_Y_output
ADRC_RANGE_output

vehicle_attitude
vehicle_velocity
vehicle_position

measurement_latency
prediction_age

target_clipped
```

---

# 46. Recommended Simulation Plots

```text
theta_x vs time

theta_y vs time

theta_x_dot vs time

theta_y_dot vs time

scale vs time

scale_dot vs time

TTC vs time

ADRC output vs time

mode vs time

confidence vs time

measurement latency vs time
```

These will be essential when tuning the estimator and ADRC controllers.

---

# 47. Main Configurable Parameters

```text
confidence_min

alpha_x
beta_x

alpha_y
beta_y

alpha_scale
beta_scale

scale_dot_min

track_center_threshold_x
track_center_threshold_y

terminal_scale_threshold

commit_scale_threshold

commit_theta_x_max
commit_theta_y_max

commit_theta_x_dot_max
commit_theta_y_dot_max

commit_confidence_min

commit_persistence_time

commit_enter_scale
commit_cancel_scale

max_measurement_age
max_prediction_age

target_loss_timeout

max_yaw_rate
max_forward_velocity
max_vertical_velocity

command_slew_rate
```

---

# 48. Initial Scale Regions

These are starting values only.

```text
scale < 0.15
    FAR region

0.15 <= scale < 0.45
    TTC APPROACH region

0.45 <= scale < 0.60
    TERMINAL ALIGN region

scale >= 0.60
    COMMIT scale condition
```

Actual values depend on:

- camera FOV,
- target size,
- target geometry,
- detector behavior,
- vehicle speed,
- simulation dynamics.

---

# 49. Recommended Development Order

## Stage 1

```text
Detector
 ↓
LOS Conversion
 ↓
Alpha-Beta Estimator
 ↓
Logging / Plots
```

Validate:

```text
theta_x
theta_y
theta_x_dot
theta_y_dot
```

## Stage 2

Add:

```text
ADRC-X
ADRC-Y
```

Test target centering only.

## Stage 3

Add:

```text
bbox
 ↓
scale
 ↓
alpha-beta filter
 ↓
scale_dot
```

## Stage 4

Add:

```text
TTC estimation
TTC validity
```

## Stage 5

Add:

```text
FAR APPROACH
```

## Stage 6

Add:

```text
TTC APPROACH
```

## Stage 7

Add:

```text
TERMINAL ALIGN
```

## Stage 8

Add simulation-only:

```text
COMMIT
```

using:

```text
scale
LOS error
LOS rate
confidence
clipping
persistence
```

## Stage 9

Add:

```text
lost-target recovery
mode blending
latency compensation
```

## Stage 10

Tune estimator, ADRC parameters, and state-machine thresholds using repeated simulation runs.

---

# 50. Final Design Summary

The system is built around three visual state groups:

```text
Horizontal LOS:
    theta_x
    theta_x_dot

Vertical LOS:
    theta_y
    theta_y_dot

Range proxy:
    scale
    scale_dot
    TTC
```

The main control mapping is:

```text
theta_x
    ↓
ADRC-X
    ↓
yaw-rate command


theta_y
    ↓
ADRC-Y
    ↓
vertical-velocity command


scale + scale_dot + TTC
    ↓
ADRC-RANGE
    ↓
forward-velocity command
```

The state-machine sequence is:

```text
SEARCH
   ↓
TRACK
   ↓
FAR APPROACH
   ↓
TTC APPROACH
   ↓
TERMINAL ALIGN
   ↓
COMMIT
   ↓
SIMULATION INTERCEPT EVENT
```

The initial COMMIT scale condition is:

```text
bbox_height / image_height > 0.60
```

but COMMIT also requires:

```text
small horizontal LOS error

small vertical LOS error

small horizontal LOS rate

small vertical LOS rate

stable tracker confidence

valid estimator

valid TTC

target not clipped

persistence for a configured period
```

This architecture provides a clean separation between perception, estimation, guidance, ADRC regulation, flight-control stabilization, mode management, and simulation instrumentation.