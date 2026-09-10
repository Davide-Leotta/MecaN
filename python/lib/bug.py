import numpy as np

class Bug:

    def __init__(self,d):
        self.target_d = d
        self.direction_count = [0 for i in range(8)]
    



def get_direction(a, b):
    point_a = np.array(a) 
    point_b = np.array(b)
    points_diff = point_b - point_a
    round_diff = np.round(points_diff,1)
    
