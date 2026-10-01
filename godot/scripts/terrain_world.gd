extends Node3D
class_name TerrainWorld

## Deterministic Terrain3D forest, road, and roadside collision obstacles.

const TERRAIN_MAP_SIZE := 256
const TERRAIN_ORIGIN := -128.0
const DECORATION_HALF_EXTENT := 61.0
const ROAD_CENTER_X := 10.0
const ROAD_LENGTH_M := 80.0
const ROAD_WIDTH_M := 3.2
const FLAT_CORRIDOR_HALF_WIDTH_M := 4.0
const DECORATION_CLEARANCE_M := 8.0
const TREE_COUNT := 120
const ROCK_COUNT := 80
const WORLD_SEED := 7

const TREE_SCENES: Array[PackedScene] = [
	preload("res://assets/nature/tree_pineDefaultA.glb"),
	preload("res://assets/nature/tree_pineDefaultB.glb"),
	preload("res://assets/nature/tree_detailed.glb"),
	preload("res://assets/nature/tree_oak.glb"),
]
const ROCK_SCENES: Array[PackedScene] = [
	preload("res://assets/nature/rock_largeA.glb"),
	preload("res://assets/nature/rock_largeC.glb"),
	preload("res://assets/nature/rock_tallA.glb"),
	preload("res://assets/nature/rock_smallE.glb"),
]

var terrain: Terrain3D
var _height_noise := FastNoiseLite.new()


func _ready() -> void:
	"""Build the landscape before cameras begin consuming the shared world."""
	_height_noise.seed = WORLD_SEED
	_height_noise.frequency = 0.035
	_height_noise.fractal_octaves = 4
	_height_noise.fractal_gain = 0.45
	_build_environment()
	_build_terrain()
	_build_road()
	_build_primary_obstacles()
	_scatter_decoration()


func terrain_height(world_x: float, world_z: float) -> float:
	"""Return deterministic terrain height with a flat zero-height road corridor."""
	var distance_from_road := absf(world_z)
	if distance_from_road <= FLAT_CORRIDOR_HALF_WIDTH_M:
		return 0.0
	var blend := smoothstep(FLAT_CORRIDOR_HALF_WIDTH_M, 18.0, distance_from_road)
	var broad_noise := (_height_noise.get_noise_2d(world_x, world_z) + 1.0) * 0.5
	var ridge := absf(_height_noise.get_noise_2d(world_x * 0.45 + 90.0, world_z * 0.45 - 40.0))
	return minf(3.0, blend * (0.35 + broad_noise * 1.65 + ridge * 0.55))


func _build_environment() -> void:
	"""Create daylight, sky, ambient light, and distance fog."""
	var environment_node := WorldEnvironment.new()
	environment_node.name = "ForestEnvironment"
	var environment := Environment.new()
	var sky := Sky.new()
	var sky_material := ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color(0.16, 0.37, 0.66)
	sky_material.sky_horizon_color = Color(0.72, 0.82, 0.86)
	sky_material.ground_bottom_color = Color(0.08, 0.10, 0.07)
	sky_material.ground_horizon_color = Color(0.48, 0.55, 0.42)
	sky.sky_material = sky_material
	environment.sky = sky
	environment.background_mode = Environment.BG_SKY
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	environment.ambient_light_energy = 0.75
	environment.fog_enabled = true
	environment.fog_light_color = Color(0.72, 0.78, 0.76)
	environment.fog_density = 0.004
	environment_node.environment = environment
	add_child(environment_node)

	var sun := DirectionalLight3D.new()
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-52, -28, 0)
	sun.light_color = Color(1.0, 0.94, 0.82)
	sun.light_energy = 1.25
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 80.0
	add_child(sun)


func _build_terrain() -> void:
	"""Create Terrain3D, its texture assets, height region, and full collision."""
	terrain = Terrain3D.new()
	terrain.name = "ForestTerrain3D"
	terrain.data_directory = "res://terrain_data"
	add_child(terrain)
	# Four 128 m regions keep the height data centered on the world origin.
	terrain.region_size = Terrain3D.SIZE_128
	terrain.vertex_spacing = 1.0

	terrain.material.world_background = Terrain3DMaterial.NONE
	terrain.material.auto_shader = true
	terrain.material.set_shader_param("auto_slope", 28.0)
	terrain.material.set_shader_param("blend_sharpness", 0.92)
	terrain.assets = Terrain3DAssets.new()
	terrain.assets.set_texture(0, _terrain_texture(
		"Forest floor",
		"res://assets/terrain/forest_leaves_04_albedo_height_1k.png",
		"res://assets/terrain/forest_leaves_04_normal_roughness_1k.png",
		0.16,
	))
	terrain.assets.set_texture(1, _terrain_texture(
		"Rocky soil",
		"res://assets/terrain/rocky_terrain_02_albedo_height_1k.png",
		"res://assets/terrain/rocky_terrain_02_normal_roughness_1k.png",
		0.12,
	))

	var height_map := Image.create_empty(TERRAIN_MAP_SIZE, TERRAIN_MAP_SIZE, false, Image.FORMAT_RF)
	for pixel_x in TERRAIN_MAP_SIZE:
		for pixel_z in TERRAIN_MAP_SIZE:
			var world_x := TERRAIN_ORIGIN + float(pixel_x)
			var world_z := TERRAIN_ORIGIN + float(pixel_z)
			height_map.set_pixel(pixel_x, pixel_z, Color(terrain_height(world_x, world_z), 0, 0, 1))
	terrain.data.import_images([height_map, null, null], Vector3(TERRAIN_ORIGIN, 0, TERRAIN_ORIGIN))
	terrain.collision.mode = Terrain3DCollision.FULL_GAME


func _terrain_texture(asset_name: String, albedo_path: String, normal_path: String, uv_scale: float) -> Terrain3DTextureAsset:
	"""Load one prepacked Terrain3D albedo/height and normal/rough texture set."""
	var albedo_source: Texture2D = load(albedo_path)
	var normal_source: Texture2D = load(normal_path)

	var texture_asset := Terrain3DTextureAsset.new()
	texture_asset.name = asset_name
	texture_asset.albedo_texture = albedo_source
	texture_asset.normal_texture = normal_source
	texture_asset.uv_scale = uv_scale
	texture_asset.detiling_rotation = 0.08
	texture_asset.detiling_shift = 0.08
	return texture_asset


func _build_road() -> void:
	"""Preserve the original road dimensions, elevation, and yellow markings."""
	_add_box(Vector3(ROAD_CENTER_X, 0.02, 0), Vector3(ROAD_LENGTH_M, 0.05, ROAD_WIDTH_M), Color(0.12, 0.13, 0.14), "Road")
	for x in range(-25, 50, 5):
		_add_box(Vector3(x, 0.06, 0), Vector3(2.0, 0.012, 0.08), Color(0.95, 0.82, 0.28), "RoadMarking")


func _build_primary_obstacles() -> void:
	"""Replace the original roadside boxes with collidable trees and rocks."""
	for index in range(9):
		var x := -20.0 + float(index) * 8.0
		_add_tree_obstacle(Vector3(x, terrain_height(x, -6.0), -6.0), index)
		_add_rock_obstacle(Vector3(x - 2.0, terrain_height(x - 2.0, 6.0), 6.0), index)


func _add_tree_obstacle(position: Vector3, variant: int) -> void:
	"""Add one visible tree and a conservative trunk collision proxy."""
	var visual := TREE_SCENES[variant % TREE_SCENES.size()].instantiate()
	visual.name = "RoadsideTree%02d" % variant
	visual.position = position
	visual.scale = Vector3.ONE * (4.8 + float(variant % 3) * 0.35)
	visual.rotation.y = float(variant) * 1.73
	_apply_safe_nature_materials(visual, true)
	add_child(visual)
	_add_cylinder_obstacle(position, 0.58, 5.2, "TreeCollision%02d" % variant)


func _add_rock_obstacle(position: Vector3, variant: int) -> void:
	"""Add one visible rock and a compact collision proxy."""
	var visual := ROCK_SCENES[variant % ROCK_SCENES.size()].instantiate()
	visual.name = "RoadsideRock%02d" % variant
	visual.position = position
	visual.scale = Vector3.ONE * (3.6 + float(variant % 3) * 0.4)
	visual.rotation.y = float(variant) * 0.91
	_apply_safe_nature_materials(visual, false)
	add_child(visual)
	_add_box_obstacle(position + Vector3(0, 0.65, 0), Vector3(1.8, 1.3, 1.6), "RockCollision%02d" % variant)


func _scatter_decoration() -> void:
	"""Batch deterministic collision-free forest decoration through Terrain3D."""
	var tree_asset := Terrain3DMeshAsset.new()
	tree_asset.name = "Decorative pines"
	tree_asset.scene_file = TREE_SCENES[0]
	tree_asset.material_override = _nature_material(Color(0.10, 0.32, 0.14))
	tree_asset.lod0_range = 72.0
	tree_asset.lod1_range = 105.0
	terrain.assets.set_mesh_asset(0, tree_asset)

	var rock_asset := Terrain3DMeshAsset.new()
	rock_asset.name = "Decorative rocks"
	rock_asset.scene_file = ROCK_SCENES[0]
	rock_asset.material_override = _nature_material(Color(0.32, 0.36, 0.34))
	rock_asset.lod0_range = 62.0
	rock_asset.lod1_range = 90.0
	terrain.assets.set_mesh_asset(1, rock_asset)

	var rng := RandomNumberGenerator.new()
	rng.seed = WORLD_SEED
	terrain.instancer.add_transforms(0, _decoration_transforms(rng, TREE_COUNT, true))
	terrain.instancer.add_transforms(1, _decoration_transforms(rng, ROCK_COUNT, false))


func _decoration_transforms(rng: RandomNumberGenerator, count: int, trees: bool) -> Array[Transform3D]:
	"""Return transforms outside the protected road corridor."""
	var transforms: Array[Transform3D] = []
	while transforms.size() < count:
		var x := rng.randf_range(-DECORATION_HALF_EXTENT, DECORATION_HALF_EXTENT)
		var z := rng.randf_range(-DECORATION_HALF_EXTENT, DECORATION_HALF_EXTENT)
		if absf(z) < DECORATION_CLEARANCE_M:
			continue
		var scale_value := rng.randf_range(3.4, 5.7) if trees else rng.randf_range(1.8, 4.2)
		var basis := Basis(Vector3.UP, rng.randf_range(0.0, TAU)).scaled(Vector3.ONE * scale_value)
		transforms.append(Transform3D(basis, Vector3(x, terrain_height(x, z), z)))
	return transforms


func _apply_safe_nature_materials(node: Node, tree: bool) -> void:
	"""Desaturate red-prone source materials so vision isolates the target cube."""
	if node is MeshInstance3D:
		var mesh_instance := node as MeshInstance3D
		for surface_index in mesh_instance.mesh.get_surface_count():
			var source := mesh_instance.mesh.surface_get_material(surface_index)
			var source_name := source.resource_name.to_lower() if source else ""
			var color := Color(0.32, 0.36, 0.34)
			if tree and ("leaf" in source_name or "grass" in source_name):
				color = Color(0.10, 0.32, 0.14)
			elif tree:
				color = Color(0.28, 0.24, 0.18)
			mesh_instance.set_surface_override_material(surface_index, _nature_material(color))
	for child in node.get_children():
		_apply_safe_nature_materials(child, tree)


func _nature_material(color: Color) -> StandardMaterial3D:
	"""Return one rough neutral material that cannot satisfy the red HSV detector."""
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.92
	return material


func _add_cylinder_obstacle(base_position: Vector3, radius: float, height: float, node_name: String) -> void:
	"""Add a tagged cylindrical obstacle used by the drone collision sensor."""
	var body := StaticBody3D.new()
	body.name = node_name
	body.set_meta("collision_kind", "obstacle")
	body.position = base_position + Vector3(0, height * 0.5, 0)
	var collision := CollisionShape3D.new()
	var shape := CylinderShape3D.new()
	shape.radius = radius
	shape.height = height
	collision.shape = shape
	body.add_child(collision)
	add_child(body)


func _add_box_obstacle(center: Vector3, size: Vector3, node_name: String) -> void:
	"""Add a tagged box proxy for a roadside rock."""
	var body := StaticBody3D.new()
	body.name = node_name
	body.set_meta("collision_kind", "obstacle")
	body.position = center
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = size
	collision.shape = shape
	body.add_child(collision)
	add_child(body)


func _add_box(position: Vector3, size: Vector3, color: Color, node_name: String) -> void:
	"""Add one colored world-space box for the road or its markings."""
	var mesh := BoxMesh.new()
	mesh.size = size
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.9
	mesh.material = material
	var instance := MeshInstance3D.new()
	instance.name = node_name
	instance.mesh = mesh
	instance.position = position
	add_child(instance)
