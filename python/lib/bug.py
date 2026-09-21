from typing import Tuple, Set
from numpy.typing import NDArray
import numpy as np
import time

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

TTL = 0.5
TD = 8

class Bug:
    # r is the radius of robot, l is the max distance for obstacle avoidance, m is the moving proportional module
    def __init__(self, r: float, l: float, m: float):
        self.direction_flag = [(0,0) for i in range(TD)]
        self.r = r
        self.l = l
        self.m = m

    def add_point(self, a: NDArray[float], b: NDArray[float]):
        center_point = np.array(a)
        obstacle_point = np.array(b)
        dir_set = self.check_robot_direction(center_point,obstacle_point)
        for d in dir_set:
           self.direction_flag[d] = (1,time.time())

    def start(self,robot_pos: NDArray[float], target_pos: NDArray[float]):
        r = np.array(robot_pos)
        self.target = np.array(target_pos)
        self.d = self.get_direction(r,self.target)

    def set_target(self, target_pos: NDArray[float]):
        self.target = np.array(target_pos)
    
    def clear_flag(self):
        for i in range(TD):
            if self.direction_flag[i][0] > 0  and (time.time() - self.direction_flag[i][1]) > TTL:
                self.direction_flag[i] = (0,0) 

    def detour(self, p: NDArray[float]) -> Tuple[bool,float,float]:
        robot_pos = np.array(p)
        bugging = True
        target_dir = self.get_direction(robot_pos, self.target)
        ini_dir = self.d

        while self.direction_flag[self.d][0] > 0:
           self.d = (self.d + 1) % TD 
           if self.d == ini_dir:
            break

        if self.direction_flag[(self.d + (TD-1)) % TD][0] == 0 :
            self.d = (self.d + (TD-1)) % TD
            if self.d == target_dir:
                bugging = False
                return bugging, 0 , 0
    
        #print([d for d,_ in self.direction_flag])
        
        v = self.get_vect(self.d,self.m)

        point_to_move = robot_pos + v
    
        return bugging, point_to_move[Z], point_to_move[X]

    def get_direction(self, a: NDArray[float], b: NDArray[float]) -> int:
        points_diff = b - a
        mod = np.linalg.norm(points_diff)
        unit_vect = points_diff/mod
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
                vect = [m,0]
            case 1:
                vect = [mt,mt]
            case 2:
                vect = [0,m]
            case 3:
                vect = [-mt,mt]
            case 4:
                vect = [-m,0]
            case 5:
                vect = [-mt,-mt]
            case 6:
                vect = [0,-m]
            case 7:
                vect = [mt,-mt]
        return vect 
    
    def check_robot_direction(self, center_point: NDArray[float], obstacle_point: NDArray[float]) -> Set[int]:
        dir_set = set()
        dir_center = self.get_direction(center_point,obstacle_point)
        if np.linalg.norm(obstacle_point - center_point) < self.l:
            dir_set.add(dir_center)

        left_v = self.get_vect((dir_center + 2)%TD, self.r)
        if np.linalg.norm(obstacle_point - center_point + left_v) < self.l:
            dir_set.add(self.get_direction(center_point + left_v, obstacle_point))

        right_v = self.get_vect((dir_center + (TD - 2))%TD, self.r)
        if np.linalg.norm(obstacle_point - center_point + right_v) < self.l:
            dir_set.add(self.get_direction(center_point + right_v, obstacle_point))

        return dir_set
