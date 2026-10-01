# Godot interactive attempt controls

`--interactive` opens the live telemetry figure paused with Start and Restart
buttons. Start advances PyBullet; Restart pauses, restores all Python flight
state and the received Godot poses, clears the collision latch, and waits for
Start again. The OpenCV frame viewer stays responsive while paused.

Each reset begins a new numbered attempt folder below the run directory. The
attempt owns its video, CSV, plot, and summary. Python remains responsible for
flight movement; Godot receives a reset flag alongside the initial pose so its
display and collision trigger return to their initial state.

## Persistent completion state

An interactive session remains alive after success, abort, obstacle contact,
or timeout. The runner finalizes the attempt artifacts, pauses physics, and
publishes a completed control state without closing PyBullet or the Godot
bridge. In that state Start and Pause are disabled while Restart and Stop
remain available. Restart restores all attempt state and immediately starts
the next numbered attempt; Stop returns the last result and performs normal
backend cleanup. Non-interactive runs retain their one-shot exit behavior.

Validation: unit-test control state changes and bridge reset payloads; manually
confirm that Start waits, Restart resets both windows, and attempts have
distinct output folders. Run an attempt through post-impact completion and
verify that PyBullet and the Godot bridge remain connected, Restart launches a
fresh attempt, and Stop is the action that closes the session.
