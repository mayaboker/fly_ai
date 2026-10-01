# Design plan index

Every material design or implementation plan belongs in this directory and
must be added to this index before its code changes begin.

Statuses:

- **Planned**: accepted design that has not been implemented.
- **In progress**: implementation work has started.
- **Implemented**: the design is present in the codebase.
- **Superseded**: a later plan replaced the design.

After implementation, first commit the code and its design documentation.
Then use a second documentation-only commit to replace `Pending` with the
implementation commit hash; a commit cannot contain its own final hash.

| Plan | Summary | Status | Implementation commit | Superseded by |
| --- | --- | --- | --- | --- |
| [Godot/PyBullet renderer scene](godot-pybullet-renderer-scene.md) | Render PyBullet-owned poses and report Godot collisions. | Implemented | Unknown | — |
| [Godot OpenCV preview](godot-opencv-preview.md) | Show the Godot FPV frame in a separate OpenCV window. | Superseded | Unknown | [Godot FPV HUD](godot-hud-overlay.md) |
| [Godot live telemetry plot](godot-live-telemetry-plot.md) | Show the existing telemetry plots during Godot runs. | Implemented | Unknown | — |
| [Godot interactive attempt controls](godot-interactive-attempt-controls.md) | Start and reset interactive attempts. | Implemented | Unknown | — |
| [Godot video frame normalization](godot-video-frame-size.md) | Resize Godot frames for the configured video writer. | Implemented | Unknown | — |
| [VS Code Godot task](vscode-godot-task.md) | Start Godot before the Python simulation. | Implemented | Unknown | — |
| [Early TTC descent feed-forward](ttc-feedforward-descent.md) | Descend safely while TTC is not yet available. | Implemented | Unknown | — |
| [Godot FPV HUD](godot-hud-overlay.md) | Draw live flight telemetry and the detected bbox over the Godot FPV inset. | Implemented | `7e56252` | — |
| [Terrain3D forest world](terrain3d-forest-world.md) | Replace the flat placeholder scenery with a deterministic forest landscape. | Implemented | `c3bb441` | — |
