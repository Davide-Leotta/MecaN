import cv2
import numpy as np
from lib.dds import *

dds = DDS()
dds.start()

dds.subscribe(["colliding", "obstacle_pos_z", "obstacle_pos_x"])

larghezza, altezza = 1920, 1080
immagine = np.zeros((altezza, larghezza, 3), dtype=np.uint8)

scala = 50
centro_x = larghezza // 2
centro_z = altezza // 2

while True:
    is_colliding = dds.read("colliding")

    if is_colliding:
        obstacle_pos_z = dds.read("obstacle_pos_z")
        obstacle_pos_x = dds.read("obstacle_pos_x")

        pz = int(obstacle_pos_z * scala) + centro_z
        px = int(obstacle_pos_x * scala) + centro_x

        if 0 <= px < larghezza and 0 <= pz < altezza:
            cv2.circle(immagine, (px, pz), 5, (0, 0, 255), -1)

    cv2.imshow("SLAM", immagine)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
dds.stop()