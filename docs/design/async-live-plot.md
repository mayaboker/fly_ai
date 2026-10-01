# Asynchronous live telemetry plotting

## Intent

Prevent Matplotlib rendering from blocking the 240 Hz simulation loop while
preserving the existing six-chart live view and interactive attempt controls.
Plotting is isolated from flight behavior; pacing changes remain separate.

## Design

A spawned worker process owns Matplotlib and refreshes at 2 Hz. The simulation
writes only chart-required numeric samples to an append-only shared-memory
matrix and atomically publishes the completed row count. The worker copies
only new rows, so the growing flight history is never pickled between
processes. A generation counter resets the view between attempts.

Godot interactive runs expose Start, Pause, Restart, and Stop in a compact,
Gazebo-inspired icon toolbar anchored to the bottom-left of the viewport.
Play, pause, reset, and stop symbols use tooltips and state-aware color cues so
the controls remain clear without text labels. Commands share the existing
Godot event UDP channel and Python publishes the current interactive state
back with renderer packets.
Non-Godot interactive runs retain plot-window controls through a small bounded
command queue; telemetry never uses that queue.

## Validation

Test shared-memory publication, incremental reads, restart generations,
capacity handling, process cleanup, and mixed Godot event routing. Compare
matched plot-free and plotted Godot missions; plot-enabled real-time factor
must remain within 90 percent of the plot-free result and publication must
remain below 1 ms at p95.
