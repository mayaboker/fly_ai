# CLI target distance

## Intent

Restore 30 m as the default distance from the configured launch point to the
simulation target, while allowing individual runs to override that distance
without editing YAML. Distance is measured along world positive X; TTC and
guidance remain image-only and do not receive this metric value.

## Interface and precedence

`fly-smart --target-distance-m DISTANCE` accepts a finite positive number.
After loading the selected scenario, the CLI resolves the effective target
center as `(launch_x + DISTANCE, configured_target_y, configured_target_z)`.
The CLI value overrides `simulation.scene.target_center`; omission preserves
the loaded configuration. The primary scenario, template, Python dataclass,
and Godot pre-pose fallback use X=24.75 m, which is 30 m ahead of the default
launch X=-5.25 m.

The effective configuration is used to create/reset the PyBullet target and is
sent to Godot with the existing pose packet, so the truck visual and collision
move together. `settings.json` records the effective center and, when present,
the requested CLI override.

## Validation

Test default and custom launch positions, preservation of target Y/Z, CLI
precedence, omission behavior, and rejection of zero, negative, NaN, and
infinite distances. Run the standard tests, self-check, Terrain3D validation,
and a default Godot mission; its summary and settings must report target center
`[24.75, 0.0, 1.0]`.

Implementation evidence: 28 tests, the flight-stack self-check, and Terrain3D
structural validation pass. The default Godot mission reported target center
`[24.75, 0.0, 1.0]`, contacted the truck at 10.258 s, and completed at 13.258
s with a 7.426 m/s impact. A zero-duration CLI resolution check using
`--target-distance-m 50` recorded the requested override and effective target
center `[44.75, 0.0, 1.0]` in both settings and summary output.
