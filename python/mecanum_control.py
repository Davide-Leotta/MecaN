from lib.system import *
from lib.dds import *
from lib.time import *
from lib.dataplot import *
from lib.system import *
from lib.bug import *
from collections import deque
import numpy as np
import math
import time

PLTS_N = 10
plts = [DataPlotter() for i in range(PLTS_N)]

plts[0] = DataPlotter()
plts[0].set_x("Time ")
plts[0].add_y("posZ", "Position Z")

plts[1] = DataPlotter()
plts[1].set_x("Time ")
plts[1].add_y("posX", "Position X")
#plts[1].add_y("target", "target")

plts[2] = DataPlotter()
plts[2].set_x("Time")
plts[2].add_y("ang", "Rotation")
#plts[2].add_y("target", "target")

plts[3] = DataPlotter()
plts[3].set_x("Time")
plts[3].add_y("velZ", "Velocity Z")
#plts[3].add_y("virtual","virtual")

plts[4] = DataPlotter()
plts[4].set_x("Time")
plts[4].add_y("velX", "Velocity X")
#plts[4].add_y("virtual","virtual")

plts[5] = DataPlotter()
plts[5].set_x("Time")
plts[5].add_y("velAng", "Rotational Velocity")
#plts[5].add_y("virtual","virtual")

plts[6] = DataPlotter()
plts[6].set_x("Time")
plts[6].add_y("w1", "FrontLeft Motor")


plts[7] = DataPlotter()
plts[7].set_x("Time")
plts[7].add_y("w2", "FrontRight Motor")


plts[8] = DataPlotter()
plts[8].set_x("Time")
plts[8].add_y("w3", "ReerRight Motor")


plts[9] = DataPlotter()
plts[9].set_x("Time")
plts[9].add_y("w4", "ReerLeft Motor")

dds = DDS()
dds.start()
dds.subscribe(["posZ", "posX", "ang", "velZ","velX","velAng", "colliding", "obstacle_pos_z", "obstacle_pos_x"])

bug = Bug()
bugging = False

TTL = 4.0

valid_points = deque()

target_pos_x = 10
target_pos_z = 10
k_att = 0.2
k_rep = 4
rho_0 = 2

robot = MecanumController(2.5, 2, 50, 0.15, 1)
posCon = PositionController(0.5, 0.5, 0.1)


t = Time()
t.start()

while t.get() < 120:
    delta_t = t.elapsed()

    is_colliding = dds.read("colliding")
    robot_pos_z = dds.wait("posZ")
    robot_pos_x = dds.wait("posX")
    ang = dds.wait("ang")

    now = time.time()

   
    F_rep_x = 0
    F_rep_z = 0

    for expire, obstacle_pos_x, obstacle_pos_z in valid_points:
        dist = math.hypot(robot_pos_x - obstacle_pos_x, robot_pos_z - obstacle_pos_z)
        if 0 < dist < rho_0:
            F_rep_x += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 3)) * (robot_pos_x - obstacle_pos_x))
            F_rep_z += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 3)) * (robot_pos_z - obstacle_pos_z))

    robot_F_z = k_att * (target_pos_z - robot_pos_z) + F_rep_z
    robot_F_x = k_att * (target_pos_x - robot_pos_x) + F_rep_x
    
    valid_points = [p for p in valid_points if p[0] >= now]
    bug.clear_flag()

    #append new obstacles
    if is_colliding:
        obstacle_pos_z = dds.read("obstacle_pos_z")
        obstacle_pos_x = dds.read("obstacle_pos_x")
        bug.add_point([robot_pos_z,robot_pos_x],[obstacle_pos_z,obstacle_pos_x])
        valid_points.append((now + TTL, obstacle_pos_x, obstacle_pos_z))

    if abs(robot_F_z) < 0.1 and abs(robot_F_x) < 0.1 and (not (bugging)):
        bug.start([robot_pos_z,robot_pos_x],[target_pos_z,target_pos_x])
        bugging = True
    #pop expired points

    if bugging:
        bugging, robot_target_pos_z, robot_target_pos_x = bug.detour([robot_pos_z,robot_pos_x])
    else:
        robot_target_pos_z = robot_pos_z + robot_F_z
        robot_target_pos_x = robot_pos_x + robot_F_x

    target_p = np.array([robot_target_pos_z - robot_pos_z, robot_target_pos_x - robot_pos_x, 0 - ang])
    vel_target = posCon.evaluate(delta_t,target_p)

    velZ = dds.wait("velZ")
    velX = dds.wait("velX")
    velAng = dds.wait("velAng")

    vel_sensor = np.array([velZ, velX, velAng])
    robot.set_target(vel_target)
    w = robot.evaluate(delta_t, vel_sensor)

    dds.publish("w1", w[0], dds.DDS_TYPE_FLOAT)
    dds.publish("w2", w[1], dds.DDS_TYPE_FLOAT)
    dds.publish("w3", w[2], dds.DDS_TYPE_FLOAT)
    dds.publish("w4", w[3], dds.DDS_TYPE_FLOAT)

    plts[0].append_x(t.get())
    plts[0].append_y("posZ", robot_pos_z)
    #plts[0].append_y("target", -15)

    plts[1].append_x(t.get())
    plts[1].append_y("posX", robot_pos_x)
    #plts[1].append_y("target", -5)

    plts[2].append_x(t.get())
    plts[2].append_y("ang", ang)
    #plts[2].append_y("target", np.rad2deg(1))

    plts[3].append_x(t.get())
    plts[3].append_y("velZ", velZ)
    #plts[3].append_y("virtual", vz)

    plts[4].append_x(t.get())
    plts[4].append_y("velX", velX)
    #plts[4].append_y("virtual", vx)

    plts[5].append_x(t.get())
    plts[5].append_y("velAng", velAng)
    #plts[5].append_y("virtual", va)

    plts[6].append_x(t.get())
    plts[6].append_y("w1", w[0])

    plts[7].append_x(t.get())
    plts[7].append_y("w2", w[1])

    plts[8].append_x(t.get())
    plts[8].append_y("w3", w[2])

    plts[9].append_x(t.get())
    plts[9].append_y("w4", w[3])
    
dds.publish("w1", 0, dds.DDS_TYPE_FLOAT)
dds.publish("w2", 0, dds.DDS_TYPE_FLOAT)
dds.publish("w3", 0, dds.DDS_TYPE_FLOAT)
dds.publish("w4", 0, dds.DDS_TYPE_FLOAT)

dds.stop()
plot_multiple(plts,figsize=(5,4))
