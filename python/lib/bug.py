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
M = 1
L = 2.5
TTL = 1.5

class Bug:
    def __init__(self):
        self.direction_flag = [(0,0) for i in range(8)]

    def add_point(self,a,b):
        d = get_direction(a,b)
        v = get_vect(d)
        if np.linalg.norm(np.array(b) - np.array(a) + v) < L:
                self.direction_flag[(d+7)%8] = (1,time.time())
                self.direction_flag[d] = (1,time.time())
                self.direction_flag[(d+1)%8] = (1,time.time())

    def start(self,robot_pos, target_pos):
        self.d = get_direction(robot_pos,target_pos)
        self.t_p = target_pos

    def set_target(self,target_pos):
        self.t_p = target_pos
    
    def clear_flag(self):
        for i in range(8):
            if self.direction_flag[i][0] > 0  and (time.time() - self.direction_flag[i][1]) > TTL:
                self.direction_flag[i] = (0,0) 

    def detour(self,p):
        r_p = np.array(p)
        bugging = True
        t_direction = get_direction(r_p, self.t_p)
        ini_d = self.d

        while self.direction_flag[self.d][0] > 0:
           self.d = (self.d + 1) % 8 
           if self.d == ini_d:
            break

        if self.direction_flag[(self.d + 7) % 8][0] == 0:
            self.d = (self.d + 7) % 8
            if self.d == t_direction:
                bugging = False
                return bugging, 0 , 0
    
        print([d for d,_ in self.direction_flag])
        
        v = get_vect(self.d)

        point_to_move = r_p + [u*2 for u in v]
    
        return bugging, point_to_move[Z], point_to_move[X]


def get_direction(a, b):
    point_a = np.array(a) 
    point_b = np.array(b)
    points_diff = point_b - point_a
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

def get_vect(d):
    vect = np.array([0,0])
    match d:
        case 0:
            vect = [M,0]
        case 1:
            vect = [M,M]
        case 2:
            vect = [0,M]
        case 3:
            vect = [-M,M]
        case 4:
            vect = [-M,0]
        case 5:
            vect = [-M,-M]
        case 6:
            vect = [0,-M]
        case 7:
            vect = [M,-M]
    return vect 
