# Fly Smart Godot renderer

This project is the renderer for the `godot-render` branch. It receives
PyBullet drone and target poses on UDP `127.0.0.1:9100`. Its FPV `SubViewport`
is copied as RGB8 frames into `/dev/shm/fly_smart_fpv.rgb` using the working
double-buffered format from `godot_shm_fpv`.

Python publishes the scenario camera rate to Godot. The renderer targets 30 Hz
by default with an absolute capture deadline, so GPU readback time is included
inside the frame budget instead of being followed by a full timer interval.

The world uses the bundled Terrain3D 1.0.0 addon for Godot 4.3. A deterministic
textured forest terrain stays level through the road corridor and rises gently
away from it. Selected roadside trees and rocks are collision obstacles;
additional scattered vegetation is display-only. All required addon binaries
and CC0 environment assets are included for offline use.

Run the Godot scene before Python:

```bash
godot --path godot
uv run fly-smart --godot --headless
```

Godot renders a camera fixed to the PyBullet-driven drone and its red 2 m
target cube. Python remains responsible for OpenCV red detection, TTC
filtering, guidance, motor commands, and PyBullet flight forces. For a
`--godot` run, Godot sends target and building collision events back to Python;
the target completes the mission and a building aborts it.

The top-left FPV preview is the live operator display. Python sends its
detected target bounding box and flight telemetry back over UDP, and Godot
draws them over that preview. The overlay is outside the FPV `SubViewport`, so
the shared-memory image used for detection remains unannotated.

In the Godot window, hold the right mouse button and drag to orbit the drone;
scroll to zoom. These controls affect only the spectator view, not FPV output.

Validate the generated landscape structure with:

```bash
godot --headless --path godot --script res://scripts/validate_world.gd
```

The interactive Python task opens the telemetry plot in a separate process and
shows a control panel below the FPV preview. Use **Start**, **Pause**,
**Restart**, and **Stop** there; Python remains authoritative for attempt state.
