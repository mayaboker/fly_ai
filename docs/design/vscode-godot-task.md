# VS Code Godot task

Add one VS Code shell task that starts the existing `godot/` project with the
same `godot --path godot` command documented by the repository.  Add a second
task for the documented headless Python command with its `--godot` switch.
The renderer task has no dependencies and must be started before the Python
task.

Add a compound task that runs both in dedicated terminals. Mark the renderer
as a background task whose readiness matcher completes when it prints `FPV
shared memory ready`; sequence the Python task after that matcher so it never
opens the shared-memory file too early.

Validation: parse the task JSON and confirm the installed Godot binary reports
its version.
