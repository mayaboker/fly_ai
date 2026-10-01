extends SceneTree

## Headless validation for renderer capture-rate configuration.


func _init() -> void:
	var renderer: Node = load("res://scripts/main.gd").new()
	renderer.call("_configure_capture_hz", 30)
	_assert_equal(float(renderer.get("_capture_hz")), 30.0, "valid configured rate")
	_assert_equal(int(renderer.get("_capture_period_usec")), 33333, "30 Hz period")
	renderer.call("_configure_capture_hz", 0)
	_assert_equal(float(renderer.get("_capture_hz")), 30.0, "invalid low rate fallback")
	renderer.call("_configure_capture_hz", 500)
	_assert_equal(float(renderer.get("_capture_hz")), 30.0, "invalid high rate fallback")
	renderer.call("_configure_capture_hz", "fast")
	_assert_equal(float(renderer.get("_capture_hz")), 30.0, "invalid type fallback")
	print("Godot 30 Hz capture configuration validation passed")
	renderer.free()
	quit(0)


func _assert_equal(actual: Variant, expected: Variant, label: String) -> void:
	if actual != expected:
		push_error("%s: expected %s, got %s" % [label, expected, actual])
		quit(1)
