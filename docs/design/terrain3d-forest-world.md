# Terrain3D temperate forest world

## Intent

Replace the flat green plane and blue roadside boxes with a realistic temperate
forest while preserving the straight road, markings, red target, flight
corridor, camera, telemetry HUD, and Python-owned flight physics.

The project stays on Godot 4.3 and pins Terrain3D 1.0.0. Terrain is level at
Godot height zero across the mission corridor and becomes gently rolling away
from the road, keeping the normal Godot collision surface aligned with
PyBullet's flat ground.

## World and collision design

Terrain, road, lighting, trees, and rocks belong to a dedicated world builder.
Large trees and rocks replace the existing roadside obstacle boxes and retain
simple collision proxies tagged as `obstacle`. Additional deterministic forest
decoration is visual-only and uses batched meshes with distance visibility
limits. The road and red target geometry are unchanged.

Terrain collision is enabled, so the drone's Godot trigger is shifted slightly
upward and made thinner. At the 0.05 m launch pose it no longer penetrates and
latches the ground, while its horizontal arm span and target/obstacle contact
behavior remain intact.

Natural materials must not introduce saturated red regions that compete with
the target detector. Terrain and asset sources, licenses, and transformations
are recorded alongside the bundled assets.

## Validation

Load the project under Godot 4.3's compatibility renderer, validate deterministic
world geometry and the flat corridor, run Python tests and self-check, and
inspect camera frames for false red detections. Confirm selected trees and
rocks report obstacle collisions while the red cube remains the only target.
