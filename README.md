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

Useful checks:

```bash
uv run fly-smart --self-check
uv run fly-smart --headless --config configs/scenario.yaml
uv sync --extra dev
uv run pytest -q
```

Each run is written to `outputs/ttc_runs/<run-name>/` with settings, CSV
telemetry, summary, and plots.

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
