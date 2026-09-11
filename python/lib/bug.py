import numpy as np

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
M = 3
L = 4

class Bug:
    def __init__(self):
        self.direction_count = [0 for i in range(8)]

    def add_point(self,a,b):
        d = get_direction(a,b)
        if d != -1:
            self.direction_count[d] += 1
        return d

    def rem_point(self,d):
        self.direction_count[d] -= 1
 
    def start(self,robot_pos, target_pos):
        self.d = target_direction(robot_pos,target_pos)
        self.start_d = d

    def detour(self,p):
        pos_start = np.array(p)
        vect= [0,0]
        while self.direction_count[self.d] > 0:
           self.d = (self.d + 1) % 8 
           if self.d == self.start_d:
            return False, 0, 0

        if self.direction_count[((self.d + 7) % 8)] == 0:
           self.d = (self.d + 7) % 8

        match self.d:
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
        
        point_to_move = pos_start + vect 
    
        return True, point_to_move[Z], point_to_move[X]


def get_direction(a, b):
    point_a = np.array(a) 
    point_b = np.array(b)
    points_diff = point_b - point_a
    mod = np.linalg.norm(points_diff)
    if mod > L:
        return -1
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
