extends Node3D

@onready var robot: RigidBody3D = $Robot
@onready var obstacle: StaticBody3D = $Obstacle

@onready var real_time_pos: Label = $"RealTimePos"
@onready var real_time_vel: Label = $"RealTimeVel"

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

var obstacle_direction = 1
# Called every frame. 'delta' is the elapsed time since the previous frame.
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

	pass
