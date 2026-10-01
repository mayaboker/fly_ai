extends SceneTree

## Headless structural validation for the deterministic Terrain3D forest world.


func _init() -> void:
	"""Build the world, verify its mission invariants, and exit nonzero on failure."""
	var camera := Camera3D.new()
	camera.current = true
	root.add_child(camera)
	var world := TerrainWorld.new()
	root.add_child(world)
	await process_frame

	_assert_close(world.terrain_height(-30.0, 0.0), 0.0, "road start must stay level")
	_assert_close(world.terrain_height(44.75, 3.9), 0.0, "target corridor must stay level")
	if world.terrain_height(30.0, 30.0) <= 0.0:
		_fail("terrain outside the corridor must have positive relief")
	if world.get_node_or_null("ForestTerrain3D") == null:
		_fail("Terrain3D node was not created")
	if _count_named(world, "RoadsideTree") != 9:
		_fail("expected 9 collidable roadside trees")
	if _count_named(world, "RoadsideRock") != 9:
		_fail("expected 9 collidable roadside rocks")
	if _count_named(world, "TreeCollision") != 9:
		_fail("expected 9 tree collision proxies")
	if _count_named(world, "RockCollision") != 9:
		_fail("expected 9 rock collision proxies")

	print("Terrain3D forest world validation passed")
	world.queue_free()
	camera.queue_free()
	await process_frame
	quit(0)


func _count_named(parent: Node, prefix: String) -> int:
	"""Count direct children whose names begin with a stable prefix."""
	var count := 0
	for child in parent.get_children():
		if str(child.name).begins_with(prefix):
			count += 1
	return count


func _assert_close(value: float, expected: float, message: String) -> void:
	"""Fail when two terrain heights differ by more than floating-point noise."""
	if not is_equal_approx(value, expected):
		_fail(message + ": got " + str(value))


func _fail(message: String) -> void:
	"""Report one validation failure and terminate immediately."""
	push_error(message)
	quit(1)
