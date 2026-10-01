# Fly Smart agent guide

## Projects

This repository has two cooperating projects:

- **Python flight stack** (`src/fly_smart`): reusable mission, TTC, guidance,
  trajectory, state-fusion, PID, and RGB red-target detection logic. Keep this
  core independent of PyBullet, Godot, and simulation modules.
- **Simulation adapters** (`src/fly_smart/simulation`): PyBullet physics and
  synthetic sensors, scenario loading, telemetry, rendering, the CLI, and the
  Godot bridge. `fly-smart` runs this package.
- **Godot renderer** (`godot`): visual renderer only. Python remains the
  physics and control authority. It receives poses on UDP `127.0.0.1:9100`
  and writes RGB frames to `/dev/shm/fly_smart_fpv.rgb`.

Legacy modules such as `fly_smart.cli` and `fly_smart.config` are compatibility
imports. Put new simulation code in `fly_smart.simulation`, not beside the
production core.

## Working conventions

- Keep core modules free of PyBullet, Godot, and imports from
  `fly_smart.simulation`.
- Put scenario YAML files in `configs/` and tests in `tests/`.
- Add docstrings to every module and class, plus methods whose purpose,
  inputs, side effects, or return value are not obvious from their signature.
- Save every design and implementation plan in `docs/design/` before making
  the related code changes.
- Maintain a release document for material changes between versions. Organize
  every release entry into separate **Logic** and **Simulation** sections;
  create the document only when a release is being prepared.
- Track and review every change to TTC estimation or flight-guidance behavior.
  Before changing either, add or update a detailed document in `docs/design/`
  explaining the intent, algorithm or state-machine change, tuning impact, and
  validation evidence.
- Treat `outputs/ttc_runs/` as generated run data; do not edit it as source.
- Preserve the `fly-smart` command and compatibility imports unless a change
  explicitly authorizes breaking them.

## Validation

```bash
uv sync --extra dev
uv run pytest -q
uv run fly-smart --self-check
uv run fly-smart --headless --config configs/scenario.yaml
```

For the Godot camera path, start Godot before Python:

```bash
godot --path godot
uv run fly-smart --godot --headless --config configs/scenario.yaml
```


## Code design rules

Apply SOLID principles pragmatically: give modules and classes clear
responsibilities, separate calculation logic from simulation and UI, and keep
interfaces small. Prefer composition and introduce abstractions only when they
support an actual extension or testing need.

## Git rules

When the user asks to commit, show the proposed commit message and wait for
explicit approval before creating the commit.
