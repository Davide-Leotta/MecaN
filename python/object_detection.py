import cv2
import numpy as np
import math
import time
from collections import deque
from lib.dds import *

dds = DDS()
dds.start()

dds.subscribe(["colliding", "obstacle_pos_z", "obstacle_pos_x", "posZ", "posX"])

width, height = 1000, 1000
imm = np.zeros((height, width, 3), dtype=np.uint8)
#pot_field = np.zeros((height, width, 3), dtype=np.uint8)

map_scale = 5
center_pixel_z = height // 2
center_pixel_x = width // 2

TTL = 4.0

valid_points = deque()

target_pos_x, target_pos_z = 30, 30
target_pixel_x = target_pos_x * map_scale + center_pixel_x
target_pixel_z = target_pos_z * map_scale + center_pixel_z
k_att = 0.1
k_rep = 5

while True:
    is_colliding = dds.read("colliding")
    robot_pos_z = dds.wait("posZ")
    robot_pos_x = dds.wait("posX")

    imm.fill(0)
    #pot_field.fill(0)

    now = time.time()

    robot_pixel_z = int(robot_pos_z * map_scale) + center_pixel_z
    robot_pixel_x = int(robot_pos_x * map_scale) + center_pixel_x

    rho_0 = 20
    F_rep_x = 0
    F_rep_z = 0

    #step = 50
    #for x in range(0, width, step):
    #    for z in range(0, height, step):
    #        F_att_x = k_att * (target_x - x)
    #        F_att_z = k_att * (target_z - z)
    #        F_rep_x = 0
    #        F_rep_z = 0

    #        for expire, ox, oz, color in valid_points:
    #            if color == (0, 0, 255):
    #                dist = math.hypot(x - ox, z - oz)
    #                if 0.1 < dist < rho_0:
    #                    F_rep_x += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((x - ox)/dist))
    #                    F_rep_z += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((z - oz)/dist))

    #        F_x = F_att_x + F_rep_x
    #        F_z = F_att_z + F_rep_z

    #        start_point = (x, z)
    #        end_point = (int(x + F_x), int(z + F_z)) 

    #        cv2.arrowedLine(pot_field, start_point, end_point, (255, 255, 255), 1, tipLength=0.1)

    for expire, obstacle_pos_x, obstacle_pos_z, color in valid_points:
        if color == (0, 0, 255):
            dist = math.hypot(robot_pos_x - obstacle_pos_x, robot_pos_z - obstacle_pos_z)
            if 0.01 < dist < rho_0:
                F_rep_x += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((robot_pos_x - obstacle_pos_x)/dist))
                F_rep_z += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((robot_pos_z - obstacle_pos_z)/dist))

    robot_F_x = k_att * (target_pos_x - robot_pos_x) + F_rep_x
    robot_F_z = k_att * (target_pos_z - robot_pos_z) + F_rep_z

    robot_target_pos = (robot_pos_x + robot_F_x, robot_pos_z + robot_F_z)
    dds.publish("target_x", robot_target_pos[0], dds.DDS_TYPE_FLOAT)
    dds.publish("target_z", robot_target_pos[1], dds.DDS_TYPE_FLOAT)
    robot_target_pixel = int(robot_target_pos[0] * map_scale) + center_pixel_x, int(robot_target_pos[1] * map_scale) + center_pixel_z
    cv2.arrowedLine(imm, (robot_pixel_x, robot_pixel_z), robot_target_pixel, (0, 255, 0), 3)

    valid_points.append((now + TTL, robot_pos_x, robot_pos_z, (255, 0, 0)))

    #append new obstacles
    if is_colliding:
        obstacle_pos_z = dds.read("obstacle_pos_z")
        obstacle_pos_x = dds.read("obstacle_pos_x")

        valid_points.append((now + TTL, obstacle_pos_x, obstacle_pos_z, (0, 0, 255)))

    #pop expired points
    while valid_points and valid_points[0][0] < now:
        valid_points.popleft()

    #print points
    for expire, x, z, color in valid_points:
        cv2.circle(imm, (int(x * map_scale) + center_pixel_x, int(z * map_scale) + center_pixel_z), 2, color, -1)

    cv2.imshow("SLAM", imm)
    #cv2.imshow("Potential Field", pot_field)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
dds.stop()