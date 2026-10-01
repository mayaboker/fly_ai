# PyBullet-driven Godot FPV scene

## Intent

Make the Godot scene a literal renderer of the PyBullet flight: the drone and
the target accept their poses only from the UDP bridge, the on-screen and
shared-memory cameras follow the drone, and the target is a red 2 m cube.
PyBullet remains the source of commands and movement; Godot decides collision
outcomes for Godot-backed runs.

## Godot collision events

When launched with `--godot`, Godot owns mission collision decisions. It gives
the received drone a trigger volume and the red target plus roadside buildings
static collision volumes. The Terrain3D forest replaces those placeholder
blocks with selected collidable trees, rocks, and terrain while preserving the
same event contract. A target contact sends `target` over UDP port 9101
and completes the mission; a building contact sends `obstacle` and aborts the
mission. PyBullet still advances the vehicle, but its drone-to-target contact
query is not used on this path. The regular non-Godot run retains its existing
PyBullet collision query.

The bridge drains UDP events and the runner consumes each event once. Godot
checks its physics world at its fixed rate, so the Godot run must be paced to
the Godot process rather than running the Python simulation faster than its
collision observer. This replaces a single direct PyBullet contact query with
an asynchronous event boundary; target dimensions and obstacle spacing stay
unchanged.

Validation: run the graphical Godot task and the Godot Python task together;
verify the target event ends a clear run successfully and a building event
ends it with the collision kind as its abort reason. The PyBullet-only
headless test continues to validate the original contact behavior.

## Scene and mission geometry

The bridge sends PyBullet position and quaternion samples for both bodies.
Godot converts its axes (`x, z, -y`) and applies each sample directly. The
renderer has no fallback animation, local flight control, or local collision
authority. Godot cameras look along local `-z`; the scene rotates their mount
90 degrees about `y` so they look along PyBullet's local `+x` flight axis, and
offsets them 0.35 m in front of the received drone pose.

Before the first UDP packet, the preview target uses the default target center
after axis conversion: Godot `(24.75, 1, 0)`. Its 2 m cube then spans from
ground level to 2 m above it instead of appearing half underground.

The Godot window uses a separate spectator camera that orbits the received
drone pose. Hold the right mouse button and drag to orbit; use the wheel to
zoom. The FPV subviewport remains attached to the PyBullet flight pose, so
operator viewing controls cannot affect vision, TTC, or physics.

The on-screen FPV preview has a thin blue UI border. It belongs to the overlay
only and does not modify the shared-memory camera image consumed by Python.

The spectator view renders a lightweight quadcopter made from a center body
and four blue rotor pads. The forest uses green and neutral materials so the
red detector's low-red HSV range continues to isolate the red target. Its initial 8 m orbit distance
makes the physical drone visible without adding a model asset or changing its
PyBullet collision shape.

The visual road follows Godot `x`, which is the received PyBullet `x` flight
axis. The launch drone and red target therefore sit on its centerline, with
the target in front of the FPV camera. A top-right legend labels Godot world
axes (`x` right, `y` up, `z` out of screen); it is display-only.

For the default scenario, the launch position is `(-5.25, 0, 0.05)`. Set the
static target center to `(24.75, 0, 1)`, exactly 30 m along the positive world
x flight axis from launch. Its 2 m edge gives a red `2 × 2 × 2 m` PyBullet
collision cube and identically sized Godot rendering. This shortens the old
35.25 m initial separation by 5.25 m; TTC and guidance remain unchanged, but
their visual measurement begins at the new physical distance.

## Validation

Run the Godot scene headlessly long enough to load the scene without parser
errors, then run the maintained headless Godot scenario. Confirm the target is
detected, and PyBullet reports contact with its static cube.
