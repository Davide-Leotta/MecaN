extends Node3D

@onready var robot: RigidBody3D = $Robot
@onready var obstacle: StaticBody3D = $Obstacle

@onready var real_time_pos: Label = $"RealTimePos"
@onready var real_time_vel: Label = $"RealTimeVel"

func _ready() -> void:
	dds.subscribe("target_z")
	dds.subscribe("target_x")
	dds.subscribe("temp_target_z")
	dds.subscribe("temp_target_x")
	pass

var obstacle_direction = 1

var temp_target_arrow: MeshInstance3D
var final_target_arrow: MeshInstance3D

func _process(delta: float) -> void:
	var posX: float = robot.position.x
	var posZ: float = robot.position.z
	var ang: float = rad_to_deg(robot.rotation.y)
	var velX: float = robot.linear_velocity.x
	var velZ: float = robot.linear_velocity.z
	var velAng: float = robot.angular_velocity.y

	dds.publish("posX", dds.DDS_TYPE_FLOAT, posX)
	dds.publish("posZ", dds.DDS_TYPE_FLOAT, posZ)
	dds.publish("ang", dds.DDS_TYPE_FLOAT, ang)
	dds.publish("velX", dds.DDS_TYPE_FLOAT, velX)
	dds.publish("velZ", dds.DDS_TYPE_FLOAT, velZ)
	dds.publish("velAng", dds.DDS_TYPE_FLOAT, velAng)
	
	var pos = "%.3f" % robot.position.x + " " + "%.3f" % robot.position.z
	var vel = "%.3f" % robot.linear_velocity.x + " " + "%.3f" % robot.linear_velocity.z
	real_time_pos.text = pos
	real_time_vel.text = vel
	
	if obstacle.position.x > 15:
		obstacle_direction = -1
	if obstacle.position.x < 5:
		obstacle_direction = 1
	obstacle.position.x += obstacle_direction * 0.01
	
	var temp_target_z = dds.read("temp_target_z")
	var temp_target_x = dds.read("temp_target_x")
	if temp_target_x != null and temp_target_z != null:
		temp_target_arrow = spawn_or_update_arrow(temp_target_arrow, temp_target_x, temp_target_z, Color.BLUE)
		
	var target_z = dds.read("target_z")
	var target_x = dds.read("target_x")
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
