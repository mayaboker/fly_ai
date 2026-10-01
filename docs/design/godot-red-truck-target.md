# Godot red truck target

## Intent

Replace the placeholder red cube in the Godot renderer with a recognizable
red truck while preserving the mission's target center. Rotate the truck 90
degrees around the vertical axis so it stands across the road, and enlarge its
Godot collision volume with the visible vehicle. PyBullet remains the flight
physics authority and the non-Godot renderer retains its compatibility cube.

Place the default scenario target at X=44.75 m, 20 m beyond its original
X=24.75 m position and 50 m ahead of the launch position. Keep Y, altitude,
orientation, and dimensions unchanged. Scenario configuration remains the
authoritative target pose sent to Godot.

The later CLI target-distance design restores X=24.75 m (30 m from launch) as
the project default while retaining arbitrary per-run distances.

## Visual design and collision

Build the truck from native Godot meshes: red chassis, cargo body, cab and
hood; dark windows, grille and bumpers; lights, mirrors, wheel hubs and four
cylindrical tires. Build the visual and collision geometry beneath one truck
assembly rotated 90 degrees about Godot's Y axis. Use a 4.8 x 2.5 x 2.4 m
local collision box (length, height, width), matching the enlarged visible
vehicle body. Keep the existing target center so pose transport and mission
configuration remain compatible.

## TTC and guidance impact

The detector estimates TTC from the bounding box of red pixels rather than
known target geometry. A larger, side-on truck has a wider and less uniform
silhouette than a cube, so bbox scale and growth will change. Its enlarged
collision geometry will also move the physical contact surface toward the
approaching drone. This change does not alter detection thresholds, TTC filter
gains, commit thresholds, trajectory logic, or guidance states.

Moving the target farther away delays visual growth, tracking, commit, and
contact while giving the vehicle a longer acceleration interval. Validation
must therefore confirm that the existing 35 s mission limit is sufficient and
compare the resulting phase timing, TTC, collision position, and impact speed.

Validation must compare visibility, bbox scale/growth, TTC, phase timing,
collision time, collision position, and impact speed against the compact-truck
baseline. Any detector or guidance tuning requires separate review.

## Validation

Load the Godot project, confirm the truck is the only strongly red scene
object, inspect the FPV view, and run a complete profiled Godot mission. The
red detector must maintain target visibility, the mission must contact the
rotated truck collision volume, and RTF must remain between 0.98 and 1.02.

Implementation evidence: the profiled truck mission held RTF 1.000 and
contacted the target at 10.529 s with a 7.394 m/s impact. The 30 Hz cube
baseline contacted at 10.558 s and 7.456 m/s. Track entry remained 4.017 s;
the last pre-impact TTC changed from 0.990 s to 0.827 s as expected from the
truck silhouette. Visual inspection confirmed that the red vehicle remains
isolated against the neutral forest and road. No detector, TTC, or guidance
retuning is indicated.

Enlarged side-on validation: the full profiled Godot mission held RTF 1.000,
contacted the target at 10.517 s and position X=23.558 m, and reported a 7.4
m/s impact. The last TTC was 1.061 s. The contact position agrees with the
truck's 1.2 m half-width along the approach axis (target center X=24.75 m),
confirming that the rotated collision volume follows the visible model. An FPV
frame at 8 s confirmed a clear side-on truck silhouette and uninterrupted red
bbox detection. No detector, TTC, or guidance tuning was required.

Relocated-target validation: with the center moved to X=44.75 m, a full
profiled Godot mission held RTF 1.000 and contacted the truck at 13.112 s and
X=43.447 m with a 7.331 m/s impact. Tracking began at 4.017 s, the last TTC
was 1.144 s, and the run completed at 16.112 s including the three-second
post-impact recording. The contact coordinate remains consistent with the
rotated truck collision proxy. The unchanged 35 s mission limit has ample
margin, and no TTC or guidance tuning was required.
