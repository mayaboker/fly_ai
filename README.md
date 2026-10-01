# Fly Smart

Standalone TTC diagonal-strike simulation extracted from the course.

The `master` branch is the PyBullet-only baseline. It keeps the current
course camera, physics, sensors, guidance, telemetry, and command-line entry
point. The `godot-render` branch replaces only the renderer: PyBullet remains
the physics authority, while Godot renders the scene and publishes its FPV
camera frames through Linux shared memory.

## Run the PyBullet baseline

```bash
uv sync
uv run fly-smart --config configs/scenario.yaml
```

The default truck is 30 m ahead of launch. Override its simulation distance
for one run without editing YAML:

```bash
uv run fly-smart --godot --config configs/scenario.yaml --target-distance-m 50
```

Useful checks:

```bash
uv run fly-smart --self-check
uv run fly-smart --headless --config configs/scenario.yaml
uv sync --extra dev
uv run pytest -q
```

Each run is written to `outputs/ttc_runs/<run-name>/` with settings, CSV
telemetry, summary, and plots.

See [Simulation loop and Godot data flow](docs/simulation-loop-and-godot-flow.md)
for the timing model, process ownership, UDP messages, shared-memory camera,
and one complete physics iteration.

## Run the seven-inch trainer

```bash
uv run fly-smart --config configs/seven_inch_trainer.yaml
```

For a non-interactive run, add `--headless`.

## Branches

| Branch | Renderer | Camera source |
| --- | --- | --- |
| `master` | PyBullet GUI | PyBullet camera |
| `godot-render` | Godot | Godot FPV camera via `/dev/shm/fly_smart_fpv.rgb` |

The course repository is the source of the baseline, but it is not imported
at runtime and is not modified by this project.

## Godot renderer branch

On `godot-render`, start Godot first so it creates the shared-memory camera
buffer, then start Python:

```bash
godot --path godot
uv run fly-smart --godot --headless --config configs/scenario.yaml
```

The Python process owns PyBullet physics and control. Godot renders poses
received over UDP, provides the camera image consumed by OpenCV, and sends
target or building collision events back for `--godot` mission outcomes.

To record an offline performance profile, use the VS Code task
**Profile Fly Smart and Godot** or run:

```bash
uv run fly-smart --godot --headless --config configs/scenario.yaml --profile-performance
```

The run folder then includes per-step and one-second CSV files, a
Chrome/Perfetto trace, and an aggregate performance summary.

Live telemetry plots run in a separate process and consume incremental samples
through shared memory. In interactive Godot runs, mission controls appear in
the Godot window so Matplotlib rendering cannot pause the flight loop.

GUI and Godot runs use absolute-deadline pacing at the configured physics
rate. Short processing overruns recover on later steps, while long stalls are
rebased to avoid extended catch-up bursts.
