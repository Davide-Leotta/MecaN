from typing import Tuple
from numpy.typing import NDArray
import numpy as np
import math

S = 0
SE = 1
E = 2
NE = 3
N = 4
NW = 5
W = 6
SW = 7

Z = 0
X = 1

TD = 8

class Bug:
    # r is the radius of robot, l is the max distance for obstacle avoidance, m is the moving proportional module
    def __init__(self, r: float, l: float, m: float):
        self.direction_flag = [0 for i in range(TD)]
        self.r = r
        self.l = l
        self.m = m
        self.target = np.array([0,0])
        self.clockwise = 0

    def start(self, target, pos, rep):
        self.target = np.array(target)
        target_dir = np.array([target[0] - pos[0],target[1] - pos[1]])
        force_rep = np.array(rep)
        target_ang = np.atan2(target_dir[0],target_dir[1])
        repuls_ang = np.atan2(force_rep[0],force_rep[1])
        ang_err = (target_ang - repuls_ang + math.pi) % (2 * math.pi) - math.pi
        if ang_err > 0:
            self.clockwise = 1
        else:
            self.clockwise = -1
        

    def get_position(self, pos , queue) -> Tuple[bool,float,float]:
        robot_pos = np.array(pos)
        points = [np.array([pt[2],pt[1]]) for pt in queue if math.hypot(pt[2] - robot_pos[0],pt[1] - robot_pos[1]) < self.l]
        
        for p in points:
            for d in self.check_robot_direction(robot_pos,p):
                self.direction_flag[d] = 1
        
        dir_target = self.get_direction(self.target - robot_pos)
        bugging = False
        chs_target = dir_target
    
        for i in range(TD):
            if self.direction_flag[(dir_target + (i * self.clockwise) + TD) % TD] == 0:
                chs_target = (dir_target + (i * self.clockwise) + TD) % TD
                if chs_target != dir_target:
                    bugging = True
                break
       
        v = self.get_vect(chs_target, self.m)
        point_to_move = robot_pos + v
        
        
        self.direction_flag[(chs_target + (-1 * self.clockwise) + TD)%TD] = 0

        return bugging, point_to_move[Z], point_to_move[X]

    def get_direction(self, array: NDArray[float]) -> int:
        mod = np.linalg.norm(array)
        unit_vect = array/mod
        if unit_vect[X] > 0:
            if unit_vect[Z] > 0:
                if unit_vect[X] <= 1/2:
                    direction = S
                elif unit_vect[X] <= np.sqrt(3)/2:
                    direction = SE
                else:
                    direction = E
            else:
                if unit_vect[X] >= np.sqrt(3)/2:
                    direction = E
                elif unit_vect[X] >= 1/2:
                    direction = NE
                else:
                    direction = N
        else:
            if unit_vect[Z] < 0:
                if unit_vect[X] >= -1/2:
                    direction = N
                elif unit_vect[X] >= - np.sqrt(3)/2:
                    direction = NW
                else:
                    direction = W
            else:
                if unit_vect[X] <= - np.sqrt(3)/2:
                    direction = W
                elif unit_vect[X] <= -1/2:
                    direction = SW
                else:
                    direction = S
        return direction

    def get_vect(self, d: int, m: float) -> NDArray[float]:
        vect = np.array([0,0])
        mt = m * np.sqrt(2)/2
        match d:
            case 0:
                vect = [m, 0]
            case 1:
                vect = [mt, mt]
            case 2:
                vect = [0, m]
            case 3:
                vect = [-mt, mt]
            case 4:
                vect = [-m, 0]
            case 5:
                vect = [-mt, -mt]
            case 6:
                vect = [0, -m]
            case 7:
                vect = [mt, -mt]
        return vect 
    
    def check_robot_direction(self, center_point, obstacle_point):
        dir_set = set()
        dir_center = self.get_direction(obstacle_point - center_point)
        if np.linalg.norm(obstacle_point - center_point) < self.l:
            dir_set.add(dir_center)

        left_v = self.get_vect((dir_center + 2)%TD, self.r)
        if np.linalg.norm(obstacle_point - (center_point + left_v)) < self.l:
            dir_set.add(self.get_direction(obstacle_point - (center_point + left_v)))

        right_v = self.get_vect((dir_center + (TD - 2))%TD, self.r)
        if np.linalg.norm(obstacle_point - (center_point + right_v)) < self.l:
            dir_set.add(self.get_direction(obstacle_point - (center_point + right_v)))

        return dir_set
