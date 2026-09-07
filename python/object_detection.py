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
pot_field = np.zeros((height, width, 3), dtype=np.uint8)

scale = 5
center_z = height // 2
center_x = width // 2

TTL = 5.0

valid_points = deque()

target_x, target_z = 800, 800 
k_att = 0.1
k_rep = 20000

while True:
    is_colliding = dds.read("colliding")
    robot_pos_z = dds.wait("posZ")
    robot_pos_x = dds.wait("posX")

    imm.fill(0)
    pot_field.fill(0)

    now = time.time()

    rpz = int(robot_pos_z * scale) + center_z
    rpx = int(robot_pos_x * scale) + center_x

    step = 50
    for x in range(0, width, step):
        for z in range(0, height, step):
            F_att_x = k_att * (target_x - x)
            F_att_z = k_att * (target_z - z)
            F_rep_x = 0
            F_rep_z = 0

            rho_0 = 500

            for expire, ox, oz, color in valid_points:
                if color == (0, 0, 255):
                    dist = math.hypot(x - ox, z - oz)
                    if 0.1 < dist < rho_0:
                        F_rep_x += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((x - ox)/dist))
                        F_rep_z += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((z - oz)/dist))

            F_x = F_att_x + F_rep_x
            F_z = F_att_z + F_rep_z


            start_point = (x, z)
            end_point = (int(x + F_x), int(z + F_z)) 

            cv2.arrowedLine(pot_field, start_point, end_point, (255, 255, 255), 1, tipLength=0.1)

    for expire, ox, oz, color in valid_points:
        if color == (0, 0, 255):
            dist = math.hypot(rpx - ox, rpz - oz)
            if 0.01 < dist < rho_0:
                F_rep_x += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((rpx - ox)/dist))
                F_rep_z += k_rep * (((1/dist) - (1/rho_0)) * (1/pow(dist, 2)) * ((rpz - oz)/dist))

    robot_F_x = (target_x - rpx) + F_rep_x
    robot_F_z = (target_z - rpz) + F_rep_z

    robot_end_point = (int(rpx + robot_F_x * k_att), int(rpz + robot_F_z * k_att))
    cv2.arrowedLine(pot_field, (rpx, rpz), robot_end_point, (0, 255, 0), 3)

    valid_points.append((now + TTL, rpx, rpz, (255, 0, 0)))

    if is_colliding:
        obstacle_pos_z = dds.read("obstacle_pos_z")
        obstacle_pos_x = dds.read("obstacle_pos_x")

        opz = int(obstacle_pos_z * scale) + center_z
        opx = int(obstacle_pos_x * scale) + center_x

        valid_points.append((now + TTL, opx, opz, (0, 0, 255)))

    while valid_points and valid_points[0][0] < now:
        valid_points.popleft()

    for expire, x, z, color in valid_points:
        cv2.circle(imm, (x, z), 2, color, -1)

    cv2.imshow("SLAM", imm)
    cv2.imshow("Potential Field", pot_field)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
dds.stop()