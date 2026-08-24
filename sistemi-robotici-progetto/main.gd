extends Node3D

@onready var robot: RigidBody3D = $Robot

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	var posX: float = robot.position.x
	var posZ: float = robot.position.z
	dds.publish("posX", DDS.DDS_TYPE_FLOAT, posX)
	dds.publish("posZ", DDS.DDS_TYPE_FLOAT, posZ)
		
	pass
