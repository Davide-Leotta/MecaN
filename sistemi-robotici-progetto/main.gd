extends Node3D

@onready var robot: RigidBody3D = $Robot

@onready var real_time_pos: Label = $CanvasLayer/RealTimePos
@onready var real_time_vel: Label = $CanvasLayer/RealTimeVel
@onready var target_pos: Label = $CanvasLayer/TargetPos
@onready var path_type_label: Label = $CanvasLayer/PathType

@onready var obstacle_20: StaticBody3D = $Obstacle20
@onready var obstacle_21: StaticBody3D = $Obstacle21
@onready var obstacle_22: StaticBody3D = $Obstacle22
@onready var obstacle_23: StaticBody3D = $Obstacle23
@onready var obstacle_24: StaticBody3D = $Obstacle24
@onready var obstacle_25: StaticBody3D = $Obstacle25

var obstacles = []

func _ready() -> void:
	$CanvasLayer/SubViewportContainer/SubViewport.world_3d = get_viewport().world_3d
	dds.subscribe("target_z")
	dds.subscribe("target_x")
	dds.subscribe("tmp_target_z")
	dds.subscribe("tmp_target_x")
	dds.subscribe("path_type")
	
	obstacles = [obstacle_20, obstacle_21, obstacle_22, obstacle_23, obstacle_24, obstacle_25]
	pass

var tmp_target_arrow: MeshInstance3D
var final_target_arrow: MeshInstance3D

var direction = 1
func _process(delta: float) -> void:
	var obstacle_speed = 0.01
	if obstacles[0].position.z <= -78:
		direction  = 1
	elif obstacles[0].position.z >= -67:
		direction = -1
	for i in range(obstacles.size()):
		if i % 2 == 0:
			obstacles[i].position.z += obstacle_speed * direction
		else:
			obstacles[i].position.z -= obstacle_speed * direction

	var pos_z: float = robot.position.z
	var pos_x: float = robot.position.x
	var ang: float = rad_to_deg(robot.rotation.y)
	var vel_x: float = robot.linear_velocity.x
	var vel_z: float = robot.linear_velocity.z
	var vel_ang: float = robot.angular_velocity.y
	
	var target_z = dds.read("target_z")
	var target_x = dds.read("target_x")
	var path_type = dds.read("path_type")

	dds.publish("pos_z", dds.DDS_TYPE_FLOAT, pos_z)
	dds.publish("pos_x", dds.DDS_TYPE_FLOAT, pos_x)
	dds.publish("ang", dds.DDS_TYPE_FLOAT, ang)
	dds.publish("vel_x", dds.DDS_TYPE_FLOAT, vel_x)
	dds.publish("vel_z", dds.DDS_TYPE_FLOAT, vel_z)
	dds.publish("vel_ang", dds.DDS_TYPE_FLOAT, vel_ang)
	
	var pos = "%.3f" % robot.position.x + " " + "%.3f" % robot.position.z
	var vel = "%.3f" % robot.linear_velocity.x + " " + "%.3f" % robot.linear_velocity.z
	if target_x and target_z:
		var trg = "%.3f" % target_x + " " + "%.3f" % target_z
		target_pos.text = trg
	real_time_pos.text = pos
	real_time_vel.text = vel

	if path_type != null:
		if path_type == 0:
			path_type_label.text = "Potential Field"
		elif path_type == 1:
			path_type_label.text = "Bug-0"
	
	var tmp_target_z = dds.read("tmp_target_z")
	var tmp_target_x = dds.read("tmp_target_x")
	if tmp_target_x != null and tmp_target_z != null:
		tmp_target_arrow = spawn_or_update_arrow(tmp_target_arrow, tmp_target_x, tmp_target_z, Color.BLUE)
		
	if target_x != null and target_z != null:
		final_target_arrow = spawn_or_update_arrow(final_target_arrow, target_x, target_z, Color.GREEN)

func spawn_or_update_arrow(arrow_node: MeshInstance3D, target_x: float, target_z: float, arrow_color: Color) -> MeshInstance3D:
	if arrow_node == null:
		arrow_node = MeshInstance3D.new()
		
		var mesh = BoxMesh.new()
		arrow_node.mesh = mesh
		
		var material = StandardMaterial3D.new()
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		arrow_color.a = 0.5 
		
		material.albedo_color = arrow_color 
		arrow_node.material_override = material
		
		arrow_node.rotation.x = deg_to_rad(45)
		arrow_node.rotation.y = deg_to_rad(45)
		arrow_node.scale = Vector3(0.25, 0.25, 0.25)
		
		add_child(arrow_node)
	
	arrow_node.position = Vector3(target_x, 1.0, target_z)
	
	return arrow_node
