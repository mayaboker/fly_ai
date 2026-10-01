extends Node3D

# Linux shared-memory-backed file. Godot writes; Python maps and reads it.
const SHM_PATH := "/dev/shm/fly_smart_fpv.rgb"
const WIDTH := 640
const HEIGHT := 360
const FPS := 15.0
const HEADER_BYTES := 32
const FRAME_BYTES := WIDTH * HEIGHT * 3  # tightly packed RGB8
const POSE_PORT := 9100
const COLLISION_PORT := 9101
const FPV_MOUNT := Transform3D(Basis(Vector3.UP, -PI / 2.0), Vector3(0.35, 0.05, 0.0))
const SPECTATOR_TURN_SPEED := 0.01
const PREVIEW_POSITION := Vector2(10, 10)
const PREVIEW_SIZE := Vector2(480, 270)

var _drone: Node3D
var _target: Node3D
var _fpv_viewport: SubViewport
var _fpv_camera: Camera3D
var _display_camera: Camera3D
var _collision_sensor: Area3D
var _shm: FileAccess
var _pose_socket := PacketPeerUDP.new()
var _collision_socket := PacketPeerUDP.new()
var _active_slot := 0
var _sequence := 0
var _latest_pose: Dictionary = {}
var _collision_reported := false
var _spectator_yaw := -PI / 2.0
var _spectator_pitch := 0.35
var _spectator_distance := 8.0
var _hud_label: Label
var _target_label: Label
var _bbox_panel: Panel


func _ready() -> void:
	_build_world()
	_build_drone()
	_build_target()
	_build_cameras()
	_collision_socket.connect_to_host("127.0.0.1", COLLISION_PORT)
	var bind_error := _pose_socket.bind(POSE_PORT, "127.0.0.1")
	if bind_error != OK:
		push_error("Cannot listen for PyBullet poses on UDP %d: %s" % [POSE_PORT, bind_error])
	else:
		print("Waiting for PyBullet poses on UDP 127.0.0.1:", POSE_PORT)
	_open_shared_memory()
	if _shm != null:
		_capture_loop()
		print("FPV shared memory ready: ", SHM_PATH)


func _process(_delta: float) -> void:
	_receive_latest_pose()
	var camera_transform := _drone.global_transform * FPV_MOUNT
	_fpv_camera.global_transform = camera_transform
	_update_spectator_camera()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if event.pressed else Input.MOUSE_MODE_VISIBLE
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_spectator_distance = maxf(2.0, _spectator_distance - 2.0)
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_spectator_distance = minf(100.0, _spectator_distance + 2.0)
	elif event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		_spectator_yaw -= event.relative.x * SPECTATOR_TURN_SPEED
		_spectator_pitch = clampf(_spectator_pitch - event.relative.y * SPECTATOR_TURN_SPEED, -1.4, 1.4)


func _update_spectator_camera() -> void:
	var direction := Vector3(
		sin(_spectator_yaw) * cos(_spectator_pitch),
		sin(_spectator_pitch),
		cos(_spectator_yaw) * cos(_spectator_pitch),
	)
	_display_camera.global_position = _drone.global_position + direction * _spectator_distance
	_display_camera.look_at(_drone.global_position)


func _build_world() -> void:
	var env := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_COLOR
	settings.background_color = Color(0.50, 0.73, 0.93)
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	settings.ambient_light_color = Color(0.75, 0.82, 0.9)
	env.environment = settings
	add_child(env)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -25, 0)
	sun.light_energy = 1.5
	add_child(sun)

	_add_box(self, Vector3(10, -0.15, 0), Vector3(80, 0.3, 80), Color(0.34, 0.53, 0.29))
	_add_box(self, Vector3(10, 0.02, 0), Vector3(80, 0.05, 3.2), Color(0.22, 0.24, 0.26))
	for x in range(-25, 50, 5):
		_add_box(self, Vector3(x, 0.06, 0), Vector3(2.0, 0.01, 0.08), Color(0.95, 0.86, 0.46))
	for i in range(9):
		var x := -20.0 + float(i) * 8.0
		var height := 1.2 + float(i % 3) * 0.55
		_add_obstacle(self, Vector3(x, height * 0.5, -6), Vector3(2.1, height, 2.1), Color(0.16, 0.38, 0.72))
		_add_obstacle(self, Vector3(x - 2, height * 0.5, 6), Vector3(2.1, height, 2.1), Color(0.12, 0.28, 0.58))


func _build_drone() -> void:
	_drone = Node3D.new()
	_drone.name = "Drone"
	_drone.position = Vector3(-5.25, 0.05, 0.0)
	add_child(_drone)
	_add_box(_drone, Vector3.ZERO, Vector3(0.42, 0.16, 0.55), Color(0.82, 0.86, 0.9))
	for x in [-0.42, 0.42]:
		for z in [-0.42, 0.42]:
			_add_box(_drone, Vector3(x, 0.02, z), Vector3(0.22, 0.05, 0.22), Color(0.08, 0.35, 0.92))
	_collision_sensor = Area3D.new()
	var drone_shape := CollisionShape3D.new()
	var drone_box := BoxShape3D.new()
	drone_box.size = Vector3(0.9, 0.25, 0.9)
	drone_shape.shape = drone_box
	_collision_sensor.add_child(drone_shape)
	_collision_sensor.body_entered.connect(_on_drone_body_entered)
	_drone.add_child(_collision_sensor)


func _build_target() -> void:
	_target = Node3D.new()
	_target.name = "PyBulletTarget"
	_target.position = Vector3(24.75, 1.0, 0.0)
	add_child(_target)
	_add_box(_target, Vector3.ZERO, Vector3(2, 2, 2), Color(0.9, 0.05, 0.05))
	_add_collision_body(_target, Vector3.ZERO, Vector3(2, 2, 2), "target")


func _receive_latest_pose() -> void:
	# Drain the socket: rendering should use the newest physics sample.
	while _pose_socket.get_available_packet_count() > 0:
		var packet := _pose_socket.get_packet().get_string_from_utf8()
		var value: Variant = JSON.parse_string(packet)
		if not value is Dictionary:
			continue
		_latest_pose = value
		if value.get("reset", false):
			_collision_reported = false
			_clear_hud()
		_apply_pose(value.get("drone"), _drone)
		_apply_pose(value.get("target"), _target)
		var telemetry: Variant = value.get("telemetry")
		if telemetry is Dictionary:
			_update_hud(telemetry)


func _apply_pose(raw_pose: Variant, node: Node3D) -> void:
	if not raw_pose is Dictionary:
		return
	var position_b: Variant = raw_pose.get("p")
	var attitude_b: Variant = raw_pose.get("q")
	if not position_b is Array or not attitude_b is Array:
		return
	if position_b.size() != 3 or attitude_b.size() != 4:
		return
	# PyBullet: X right, Y forward, Z up; Godot: X right, -Z forward, Y up.
	var position_g := Vector3(float(position_b[0]), float(position_b[2]), -float(position_b[1]))
	var rotation_g := Quaternion(
		float(attitude_b[0]), float(attitude_b[2]),
		-float(attitude_b[1]), float(attitude_b[3])
	).normalized()
	node.global_transform = Transform3D(Basis(rotation_g), position_g)


func _build_cameras() -> void:
	_display_camera = Camera3D.new()
	_display_camera.name = "DroneCamera"
	add_child(_display_camera)
	_display_camera.current = true

	_fpv_viewport = SubViewport.new()
	_fpv_viewport.name = "FPVViewport"
	_fpv_viewport.size = Vector2i(WIDTH, HEIGHT)
	_fpv_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_fpv_viewport.world_3d = get_viewport().world_3d
	add_child(_fpv_viewport)

	_fpv_camera = Camera3D.new()
	_fpv_camera.name = "FPVCamera"
	_fpv_camera.projection = Camera3D.PROJECTION_PERSPECTIVE
	_fpv_camera.fov = 75.0  # vertical FOV when KEEP_HEIGHT is selected
	_fpv_camera.near = 0.05
	_fpv_camera.far = 500.0
	_fpv_viewport.add_child(_fpv_camera)
	_fpv_camera.current = true

	var overlay := CanvasLayer.new()
	add_child(overlay)
	var preview_frame := Panel.new()
	preview_frame.name = "FPVPreviewFrame"
	preview_frame.position = Vector2(6, 6)
	preview_frame.size = Vector2(488, 278)
	var preview_style := StyleBoxFlat.new()
	preview_style.bg_color = Color(0, 0, 0, 0)
	preview_style.border_color = Color(0.08, 0.55, 0.95)
	preview_style.set_border_width_all(2)
	preview_frame.add_theme_stylebox_override("panel", preview_style)
	overlay.add_child(preview_frame)
	var preview := TextureRect.new()
	preview.name = "FPVPreview"
	preview.position = PREVIEW_POSITION
	preview.size = PREVIEW_SIZE
	preview.texture = _fpv_viewport.get_texture()
	preview.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	overlay.add_child(preview)
	_build_hud(overlay)

	var axes := Label.new()
	axes.name = "WorldAxes"
	axes.text = "Y ↑\nZ ⊙   X →"
	axes.add_theme_font_size_override("font_size", 20)
	axes.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	axes.offset_left = -130
	axes.offset_top = 16
	axes.offset_right = -16
	axes.offset_bottom = 80
	overlay.add_child(axes)


func _build_hud(overlay: CanvasLayer) -> void:
	"""Build display-only controls over the FPV preview."""
	var hud_panel := Panel.new()
	hud_panel.name = "FlightHUDPanel"
	hud_panel.position = PREVIEW_POSITION + Vector2(8, 8)
	hud_panel.size = Vector2(218, 218)
	hud_panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var hud_style := StyleBoxFlat.new()
	hud_style.bg_color = Color(0.02, 0.03, 0.05, 0.58)
	hud_style.corner_radius_top_left = 5
	hud_style.corner_radius_top_right = 5
	hud_style.corner_radius_bottom_left = 5
	hud_style.corner_radius_bottom_right = 5
	hud_panel.add_theme_stylebox_override("panel", hud_style)
	overlay.add_child(hud_panel)

	_hud_label = Label.new()
	_hud_label.name = "FlightHUD"
	_hud_label.position = Vector2(8, 6)
	_hud_label.size = Vector2(202, 206)
	_hud_label.add_theme_font_size_override("font_size", 13)
	_hud_label.add_theme_color_override("font_color", Color.WHITE)
	_hud_label.add_theme_color_override("font_shadow_color", Color.BLACK)
	_hud_label.add_theme_constant_override("shadow_offset_x", 1)
	_hud_label.add_theme_constant_override("shadow_offset_y", 1)
	hud_panel.add_child(_hud_label)

	_bbox_panel = Panel.new()
	_bbox_panel.name = "TargetBoundingBox"
	_bbox_panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var bbox_style := StyleBoxFlat.new()
	bbox_style.bg_color = Color(0, 0, 0, 0)
	bbox_style.border_color = Color(1.0, 0.82, 0.1)
	bbox_style.set_border_width_all(3)
	_bbox_panel.add_theme_stylebox_override("panel", bbox_style)
	_bbox_panel.visible = false
	overlay.add_child(_bbox_panel)

	_target_label = Label.new()
	_target_label.name = "TargetStatus"
	_target_label.position = PREVIEW_POSITION + Vector2(232, 238)
	_target_label.size = Vector2(238, 24)
	_target_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_target_label.add_theme_font_size_override("font_size", 14)
	_target_label.add_theme_color_override("font_color", Color(1.0, 0.82, 0.1))
	_target_label.add_theme_color_override("font_shadow_color", Color.BLACK)
	_target_label.add_theme_constant_override("shadow_offset_x", 1)
	_target_label.add_theme_constant_override("shadow_offset_y", 1)
	overlay.add_child(_target_label)
	_clear_hud()


func _clear_hud() -> void:
	"""Remove telemetry and target annotations until Python sends fresh state."""
	if _hud_label != null:
		_hud_label.text = "Waiting for flight telemetry..."
	if _target_label != null:
		_target_label.text = "TARGET: --"
	if _bbox_panel != null:
		_bbox_panel.visible = false


func _update_hud(data: Dictionary) -> void:
	"""Format Python telemetry and align its source-image bbox with the preview."""
	var position: Variant = data.get("position_m")
	var velocity: Variant = data.get("velocity_mps")
	var lines := PackedStringArray([
		"%s   t %.2f s" % [str(data.get("phase", "--")).to_upper(), float(data.get("time_s", 0.0))],
		"pos  x %s  y %s  z %s" % [_component(position, 0), _component(position, 1), _component(position, 2)],
		"vel  x %s  y %s  z %s" % [_component(velocity, 0), _component(velocity, 1), _component(velocity, 2)],
		"alt %s m   vz %s m/s" % [_number(data.get("altitude_m"), 2), _number(data.get("vertical_velocity_mps"), 2)],
		"pitch %s / %s deg" % [_number(data.get("measured_pitch_deg"), 1), _number(data.get("commanded_pitch_deg"), 1)],
		"thrust %s N" % _number(data.get("thrust_n"), 2),
		"cmd vx %s   vz %s m/s" % [_number(data.get("command_vx_mps"), 2), _number(data.get("command_vz_mps"), 2)],
		"scale %s px" % _number(data.get("scale_px"), 1),
		"growth %s px/s" % _number(data.get("growth_px_s"), 1),
		"TTC %s s" % _number(data.get("ttc_s"), 2),
	])
	_hud_label.text = "\n".join(lines)

	var target_visible := bool(data.get("target_visible", false))
	var bbox: Variant = data.get("bbox")
	if target_visible and bbox is Array and bbox.size() == 4:
		var scale := PREVIEW_SIZE / Vector2(WIDTH, HEIGHT)
		_bbox_panel.position = PREVIEW_POSITION + Vector2(float(bbox[0]), float(bbox[1])) * scale
		_bbox_panel.size = Vector2(float(bbox[2]), float(bbox[3])) * scale
		_bbox_panel.visible = true
		_target_label.text = "TARGET LOCK  [%d, %d, %d, %d]" % [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
	else:
		_bbox_panel.visible = false
		_target_label.text = "TARGET: %s" % ("VISIBLE / ESTIMATING" if target_visible else "NOT VISIBLE")


func _component(value: Variant, index: int) -> String:
	"""Format one vector component received from Python."""
	if value is Array and value.size() > index:
		return _number(value[index], 2)
	return "--"


func _number(value: Variant, decimals: int) -> String:
	"""Format an optional telemetry number without treating null as zero."""
	if value == null or not (value is int or value is float):
		return "--"
	return ("%." + str(decimals) + "f") % float(value)


func _add_box(parent: Node, pos: Vector3, size: Vector3, color: Color) -> void:
	var mesh := BoxMesh.new()
	mesh.size = size
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	mesh.material = material
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.position = pos
	parent.add_child(instance)


func _add_obstacle(parent: Node, pos: Vector3, size: Vector3, color: Color) -> void:
	_add_box(parent, pos, size, color)
	_add_collision_body(parent, pos, size, "obstacle")


func _add_collision_body(parent: Node, pos: Vector3, size: Vector3, kind: String) -> void:
	var body := StaticBody3D.new()
	body.set_meta("collision_kind", kind)
	body.position = pos
	var collision_shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	collision_shape.shape = box
	body.add_child(collision_shape)
	parent.add_child(body)


func _on_drone_body_entered(body: Node3D) -> void:
	if _collision_reported:
		return
	var kind: Variant = body.get_meta("collision_kind", "obstacle")
	_collision_socket.put_packet(JSON.stringify({"event": "collision", "kind": kind}).to_utf8_buffer())
	_collision_reported = true


func _open_shared_memory() -> void:
	_shm = FileAccess.open(SHM_PATH, FileAccess.WRITE_READ)
	if _shm == null:
		push_error("Cannot open %s: %s" % [SHM_PATH, FileAccess.get_open_error()])
		return
	# Header: magic, width, height, channels, active_slot, sequence, frame_bytes, reserved.
	_shm.big_endian = false
	_shm.store_buffer("GFPV".to_ascii_buffer())
	_shm.store_32(WIDTH)
	_shm.store_32(HEIGHT)
	_shm.store_32(3)
	_shm.store_32(0)
	_shm.store_32(0)
	_shm.store_32(FRAME_BYTES)
	_shm.store_32(0)
	_shm.seek(HEADER_BYTES + 2 * FRAME_BYTES - 1)
	_shm.store_8(0)
	_shm.flush()


func _capture_loop() -> void:
	while is_inside_tree() and _shm != null:
		await RenderingServer.frame_post_draw
		var image := _fpv_viewport.get_texture().get_image()
		if not image.is_empty():
			image.convert(Image.FORMAT_RGB8)
			var pixels := image.get_data()
			if pixels.size() == FRAME_BYTES:
				_write_frame(pixels)
		await get_tree().create_timer(1.0 / FPS).timeout


func _write_frame(pixels: PackedByteArray) -> void:
	var next_slot := 1 - _active_slot
	# Odd sequence means a write is underway. Python retries until it sees
	# the same even sequence before and after copying the chosen slot.
	_shm.seek(20)
	_shm.store_32(_sequence + 1)
	_shm.flush()
	_shm.seek(HEADER_BYTES + next_slot * FRAME_BYTES)
	_shm.store_buffer(pixels)
	_shm.flush()
	_shm.seek(16)
	_shm.store_32(next_slot)
	_shm.store_32(_sequence + 2)
	_shm.flush()
	_active_slot = next_slot
	_sequence += 2
