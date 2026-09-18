extends RayCast3D

var speed = 20

func _process(delta: float) -> void:
	self.rotate_y(speed * delta)
	var colliding = self.is_colliding()
	if colliding:
		dds.publish("colliding", dds.DDS_TYPE_INT, 1)
		dds.publish("obstacle_pos_z", dds.DDS_TYPE_FLOAT, self.get_collision_point().z)
		dds.publish("obstacle_pos_x", dds.DDS_TYPE_FLOAT, self.get_collision_point().x)

	else:
		dds.publish("colliding", dds.DDS_TYPE_INT, 0)
	pass
