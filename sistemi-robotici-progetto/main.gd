extends Node3D

@onready var robot: RigidBody3D = $Robot

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	var posX: float = robot.position.x
	var posZ: float = robot.position.z
	var ang: float = robot.rotation.y
	var velX: float = robot.linear_velocity.x
	var velZ: float = robot.linear_velocity.z
	var velAng: float = robot.angular_velocity.y
	dds.publish("posX", dds.DDS_TYPE_FLOAT, posX)
	dds.publish("posZ", dds.DDS_TYPE_FLOAT, posZ)
	dds.publish("ang", dds.DDS_TYPE_FLOAT, ang)
	dds.publish("velX", dds.DDS_TYPE_FLOAT, velX)
	dds.publish("velZ", dds.DDS_TYPE_FLOAT, velZ)
	dds.publish("velAng", dds.DDS_TYPE_FLOAT, velAng)

	pass
