extends Camera3D

@onready var robot: RigidBody3D = $"../Robot"

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	#position.x = robot.position.x
	#position.y = robot.position.y + 2
	#position.z = robot.position.z + 4
	pass
