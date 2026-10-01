# Godot interactive attempt controls

`--interactive` opens the live telemetry figure paused with Start and Restart
buttons. Start advances PyBullet; Restart pauses, restores all Python flight
state and the received Godot poses, clears the collision latch, and waits for
Start again. The OpenCV frame viewer stays responsive while paused.

Each reset begins a new numbered attempt folder below the run directory. The
attempt owns its video, CSV, plot, and summary. Python remains responsible for
flight movement; Godot receives a reset flag alongside the initial pose so its
display and collision trigger return to their initial state.

Validation: unit-test control state changes and bridge reset payloads; manually
confirm that Start waits, Restart resets both windows, and attempts have
distinct output folders.
