from lib.system import *
from lib.dds import *
from lib.time import *
from lib.dataplot import *
from lib.system import *
from lib.bug import *
from lib.pot_field import *
from collections import deque
import numpy as np
import math
import time

PLTS_N = 3
plts = [DataPlotter() for i in range(PLTS_N)]

plts[0] = DataPlotter()
plts[0].set_x("Time ")
plts[0].add_y("pos_z", "Position Z")
plts[0].add_y("target", "target")

plts[1] = DataPlotter()
plts[1].set_x("Time ")
plts[1].add_y("pos_x", "Position X")
plts[1].add_y("target", "target")

plts[2] = DataPlotter()
plts[2].set_x("Time")
plts[2].add_y("ang", "Rotation")
plts[2].add_y("target", "target")

#plts[3] = DataPlotter()
#plts[3].set_x("Time")
#plts[3].add_y("vel_z", "Velocity Z")

#plts[4] = DataPlotter()
#plts[4].set_x("Time")
#plts[4].add_y("vel_x", "Velocity X")
#
#plts[5] = DataPlotter()
#plts[5].set_x("Time")
#plts[5].add_y("vel_ang", "Rotational Velocity")
#
#plts[6] = DataPlotter()
#plts[6].set_x("Time")
#plts[6].add_y("w1", "Front Left Motor")
#
#plts[7] = DataPlotter()
#plts[7].set_x("Time")
#plts[7].add_y("w2", "Front Right Motor")
#
#plts[8] = DataPlotter()
#plts[8].set_x("Time")
#plts[8].add_y("w3", "Rear Right Motor")
#
#
#plts[9] = DataPlotter()
#plts[9].set_x("Time")
#plts[9].add_y("w4", "Rear Left Motor")

dds = DDS()
dds.start()

#subscribe to robot sensors
dds.subscribe(["pos_z", "pos_x", "ang", "vel_z", "vel_x", "vel_ang", "colliding", "obstacle_pos_z", "obstacle_pos_x"])

#iit obstacle queue
TTL = 1.0
valid_points = deque()

#init controllers
robot = MecanumController(1, 2.5, 0, 15, 0.15, 0.5)
pos_con = PositionController(3, 0 , 0, 4)
ori_con = OrientationController(0.1, 0, 0, 3)
pot_field = PotField(0.5, 8, 3) #k_att, k_rep, rho_0
bug = Bug(1, 2.5, 3)
bugging = False

#set target position
target_list = [(-12, 20, 30), (-40, 25, 90), (-70, 5, -120), (-70, -50, 90)]

#start time
t = Time()
t.start()

for global_target_pos_z, global_target_pos_x, ang_target in target_list:
    #read sensors
    pos_z = dds.wait("pos_z")
    pos_x = dds.wait("pos_x")
    ang = dds.wait("ang")

    #target visuals
    dds.publish("target_z", global_target_pos_z, dds.DDS_TYPE_FLOAT) 
    dds.publish("target_x", global_target_pos_x, dds.DDS_TYPE_FLOAT) 

    #potential field forces for while cycle
    pot_field.set_target(global_target_pos_z, global_target_pos_x)
    global_tmp_target_pos_z, global_tmp_target_pos_x = pot_field.evaluate(pos_z, pos_x, valid_points)

    while math.hypot(global_target_pos_z - pos_z, global_target_pos_x - pos_x) > 0.05 or abs((ang_target - ang + 180) % 360 - 180) > 2:
        delta_t = t.elapsed()
        now = time.time()

        #read sensors
        pos_z = dds.wait("pos_z")
        pos_x = dds.wait("pos_x")
        ang = dds.wait("ang")
        vel_z = dds.wait("vel_z")
        vel_x = dds.wait("vel_x")
        ang_vel = dds.wait("vel_ang")
        is_colliding = dds.wait("colliding")

        #append new obstacles
        if is_colliding:
            obstacle_pos_z = dds.wait("obstacle_pos_z")
            obstacle_pos_x = dds.wait("obstacle_pos_x")
            valid_points.append((now + TTL, obstacle_pos_x, obstacle_pos_z, (0, 0, 255)))

        #pop expired points
        valid_points = [p for p in valid_points if p[0] >= now]

        #check if robot should be bugging
        if math.hypot(global_tmp_target_pos_x - pos_x, global_tmp_target_pos_z - pos_z) < 0.15 and t.get() > 1 and (not bugging) and math.hypot(pos_x - global_target_pos_x, pos_z - global_target_pos_z) > 2:
            rep = pot_field.get_repulsive_force()
            bug.start([global_target_pos_z, global_target_pos_x], [pos_z, pos_x], rep)
            bugging = True

        #temporary target
        if math.hypot(global_target_pos_z - pos_z, global_target_pos_x - pos_x) < 0.5:
            global_tmp_target_pos_z = global_target_pos_z
            global_tmp_target_pos_x = global_target_pos_x
        elif bugging:
            bugging, global_tmp_target_pos_z, global_tmp_target_pos_x = bug.get_position([pos_z, pos_x], valid_points)
            dds.publish("path_type", 1, dds.DDS_TYPE_INT)
        else:
            global_tmp_target_pos_z, global_tmp_target_pos_x = pot_field.evaluate(pos_z, pos_x, valid_points)
            dds.publish("path_type", 0, dds.DDS_TYPE_INT)

        dds.publish("tmp_target_z", global_tmp_target_pos_z, dds.DDS_TYPE_FLOAT)
        dds.publish("tmp_target_x", global_tmp_target_pos_x, dds.DDS_TYPE_FLOAT)

        #evaluate linear velocity error
        global_pos_err = np.array([global_tmp_target_pos_z - pos_z, global_tmp_target_pos_x - pos_x])
        global_v_err = pos_con.evaluate(delta_t, global_pos_err) - np.array([vel_z, vel_x])
        local_v_err = global_to_local(ang, global_v_err)

        #evaluate angular velocity error
        ang_err = (ang_target - ang + 180) % 360 - 180
        ang_vel_err = ori_con.evaluate(delta_t, ang_err) - ang_vel

        #evaluate engine forces
        vel_err = np.append(local_v_err, ang_vel_err)
        w = robot.evaluate(delta_t, vel_err)

        #publish engine forces
        dds.publish("w1", w[0], dds.DDS_TYPE_FLOAT)
        dds.publish("w2", w[1], dds.DDS_TYPE_FLOAT)
        dds.publish("w3", w[2], dds.DDS_TYPE_FLOAT)
        dds.publish("w4", w[3], dds.DDS_TYPE_FLOAT)

        #update plots
        plts[0].append_x(t.get())
        plts[0].append_y("pos_z", pos_z)
        plts[0].append_y("target", global_target_pos_z)

        plts[1].append_x(t.get())
        plts[1].append_y("pos_x", pos_x)
        plts[1].append_y("target", global_target_pos_x)

        plts[2].append_x(t.get())
        plts[2].append_y("ang", ang)
        plts[2].append_y("target", ang_target)

        #plts[3].append_x(t.get())
        #plts[3].append_y("vel_z", vel_z)

        #plts[4].append_x(t.get())
        #plts[4].append_y("vel_x", vel_x)

        #plts[5].append_x(t.get())
        #plts[5].append_y("vel_ang", ang_vel)

        #plts[6].append_x(t.get())
        #plts[6].append_y("w1", w[0])

        #plts[7].append_x(t.get())
        #plts[7].append_y("w2", w[1])

        #plts[8].append_x(t.get())
        #plts[8].append_y("w3", w[2])

        #plts[9].append_x(t.get())
        #plts[9].append_y("w4", w[3])

    #stop the robot
    dds.publish("w1", 0, dds.DDS_TYPE_FLOAT)
    dds.publish("w2", 0, dds.DDS_TYPE_FLOAT)
    dds.publish("w3", 0, dds.DDS_TYPE_FLOAT)
    dds.publish("w4", 0, dds.DDS_TYPE_FLOAT)

dds.stop()
#print plots
plot_multiple(plts,figsize=(10, 10))
