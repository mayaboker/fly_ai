# Godot OpenCV preview

> Superseded by [Godot FPV HUD](godot-hud-overlay.md). The Godot window now
> owns the live camera presentation and the separate OpenCV preview is removed.

Add an explicit `--show-godot-frame` option that opens an OpenCV window for
the RGB frame read from Godot. The window shows the existing red-target
annotation and TTC data, while PyBullet remains in DIRECT mode. This keeps the
operator preview independent from the PyBullet GUI and from Godot's spectator
view.

The Godot VS Code task includes this option. Press `q` or `Esc` in the OpenCV
window to end the run. Validation is CLI argument parsing plus the maintained
Python test suite; opening a desktop window requires a graphical session.
