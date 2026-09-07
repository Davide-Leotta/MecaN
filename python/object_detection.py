import cv2
import numpy as np
import time
from collections import deque
from lib.dds import *

dds = DDS()
dds.start()

dds.subscribe(["colliding", "obstacle_pos_z", "obstacle_pos_x", "posZ", "posX"])

width, height = 1000, 1000
immagine = np.zeros((height, width, 3), dtype=np.uint8)

scala = 5
centro_z = height // 2
centro_x = width // 2

TTL = 3.0

valid_points = deque()

while True:
    is_colliding = dds.read("colliding")
    robot_pos_z = dds.wait("posZ")
    robot_pos_x = dds.wait("posX")

    now = time.time()

    rpz = int(robot_pos_z * scala) + centro_z
    rpx = int(robot_pos_x * scala) + centro_x

    valid_points.append((now + TTL, rpx, rpz, (255, 0, 0)))

    if is_colliding:
        obstacle_pos_z = dds.read("obstacle_pos_z")
        obstacle_pos_x = dds.read("obstacle_pos_x")

        opz = int(obstacle_pos_z * scala) + centro_z
        opx = int(obstacle_pos_x * scala) + centro_x

        valid_points.append((now + TTL, opx, opz, (0, 0, 255)))

    immagine.fill(0)

    while valid_points and valid_points[0][0] < now:
        valid_points.popleft()

    for expire, x, z, color in valid_points:
        cv2.circle(immagine, (x, z), 2, color, -1)

    cv2.imshow("SLAM", immagine)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
dds.stop()