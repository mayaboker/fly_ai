# Godot live telemetry plot

Add an explicit `--show-plots` option that opens the existing full telemetry
figure while PyBullet remains in DIRECT mode. It refreshes the same figure used
by the PyBullet GUI path and does not alter control, TTC, or recorded output.

The Godot VS Code task includes this option. Validation is CLI argument
parsing and the maintained Python test suite; showing a desktop plot requires
a graphical session.
