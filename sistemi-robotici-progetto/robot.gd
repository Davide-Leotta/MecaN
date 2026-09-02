extends RigidBody3D

@onready var mecanum_wheel_front_left: MeshInstance3D = $MecanumWheelFrontLeft
@onready var mecanum_wheel_front_right: MeshInstance3D = $MecanumWheelFrontRight
@onready var mecanum_wheel_rear_right: MeshInstance3D = $MecanumWheelRearRight
@onready var mecanum_wheel_rear_left: MeshInstance3D = $MecanumWheelRearLeft

@onready var real_time_pos: Label = $"../RealTimePos"

var front_left_direction = Vector3(1, 0, -1)
var front_right_direction = Vector3(1, 0, 1)
var rear_right_direction = Vector3(-1, 0, 1)
var rear_left_direction = Vector3(-1, 0, -1)

var motor_power = 4

# --- Variabili per il Debug 3D ---
var debug_mesh: ImmediateMesh
var debug_mesh_instance: MeshInstance3D

func _ready():
	# Inizializzazione della mesh per disegnare le linee di debug
	debug_mesh = ImmediateMesh.new()
	debug_mesh_instance = MeshInstance3D.new()
	debug_mesh_instance.mesh = debug_mesh
	
	var mat = StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	debug_mesh_instance.material_override = mat
	
	# Impostiamo top_level a true così la mesh non ruota insieme all'auto
	# ma rimane ancorata al sistema di coordinate globali
	debug_mesh_instance.top_level = true
	add_child(debug_mesh_instance)
	
	dds.subscribe("w1")
	dds.subscribe("w2")
	dds.subscribe("w3")
	dds.subscribe("w4")

func _process(delta: float) -> void:
	if real_time_pos:
		var string = "%.3f" % position.x + " " + "%.3f" % position.z
		real_time_pos.text = string

func _physics_process(delta):
	debug_mesh.clear_surfaces()
	
	var fl = 0.0
	var fr = 0.0
	var rr = 0.0
	var rl = 0.0
	
	if Input.is_action_pressed("ui_accept"):
		fl += 0.5; fr += 0.5; rr += 0.5; rl += 0.5

	if Input.is_action_pressed("ui_up"):
		fl += 1.0; fr -= 1.0; rr -= 1.0; rl += 1.0
		
	if Input.is_action_pressed("ui_down"):
		fl -= 1.0; fr += 1.0; rr += 1.0; rl -= 1.0
		
	if Input.is_action_pressed("ui_right"):
		fl += 1.0; fr += 1.0; rr -= 1.0; rl -= 1.0
		
	if Input.is_action_pressed("ui_left"):
		fl -= 1.0; fr -= 1.0; rr += 1.0; rl += 1.0

	fl = clamp(fl, -1.0, 1.0)
	fr = clamp(fr, -1.0, 1.0)
	rr = clamp(rr, -1.0, 1.0)
	rl = clamp(rl, -1.0, 1.0)

	if fl != 0 or fr != 0 or rr != 0 or rl != 0:
		applica_forze_motori(fl, fr, rr, rl)
		
	var w1 = dds.read("w1")
	var w2 = dds.read("w2")
	var w3 = dds.read("w3")
	var w4 = dds.read("w4")
	
	if w1 != null && w2 != null && w3 != null && w4 != null:
		applica_forze_motori(w1,w2,w3,w4)

func applica_forze_motori(fl: float, fr: float, rr: float, rl: float):
	
	# Calcoliamo la posizione locale orientata secondo la rotazione del robot
	var pos_fl = global_transform.basis * Vector3(-0.5, 0, -0.5)
	var pos_fr = global_transform.basis * Vector3(0.5, 0, -0.5)
	var pos_rr = global_transform.basis * Vector3(0.5, 0, 0.5)
	var pos_rl = global_transform.basis * Vector3(-0.5, 0, 0.5)
	
	# Calcoliamo la forza globale, applicando potenza e moltiplicatore di direzione
	var force_fl = global_transform.basis * (front_left_direction * motor_power * fl)
	var force_fr = global_transform.basis * (front_right_direction * motor_power * fr)
	var force_rr = global_transform.basis * (rear_right_direction * motor_power * rr)
	var force_rl = global_transform.basis * (rear_left_direction * motor_power * rl)
	
	# DISEGNO DEBUG DELLE FORZE
	# Troviamo la posizione assoluta di partenza nel mondo (Centro della macchina + offset della ruota)
	var start_fl = global_position + pos_fl
	var start_fr = global_position + pos_fr
	var start_rr = global_position + pos_rr
	var start_rl = global_position + pos_rl
	
	# Disegniamo le linee dalla posizione di partenza verso la direzione della forza
	draw_line(start_fl, start_fl + (force_fl * 0.1), Color.RED)
	draw_line(start_fr, start_fr + (force_fr * 0.1), Color.GREEN)
	draw_line(start_rr, start_rr + (force_rr * 0.1), Color.BLUE)
	draw_line(start_rl, start_rl + (force_rl * 0.1), Color.YELLOW)
	
	# Applichiamo le forze fisiche
	apply_force(force_fl, pos_fl)
	apply_force(force_fr, pos_fr)
	apply_force(force_rr, pos_rr)
	apply_force(force_rl, pos_rl)
	
	# Animazione ruote
	mecanum_wheel_front_left.rotate_x(fl * -0.1)
	mecanum_wheel_front_right.rotate_x(fr * 0.1)
	mecanum_wheel_rear_right.rotate_x(rr * 0.1)
	mecanum_wheel_rear_left.rotate_x(rl * -0.1)

# Funzione per disegnare linee nello spazio 3D
func draw_line(inizio: Vector3, fine: Vector3, colore: Color):
	debug_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	debug_mesh.surface_set_color(colore)
	debug_mesh.surface_add_vertex(inizio)
	debug_mesh.surface_set_color(colore)
	debug_mesh.surface_add_vertex(fine)
	debug_mesh.surface_end()
