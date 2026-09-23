from typing import Deque, Tuple
from numpy.typing import NDArray
from lib.system import *
import math


class PotField:
    def __init__(self, k_att: float, k_rep: float, rho_0: float, target_pos_z: Optional[float] = None, target_pos_x: Optional[float] = None):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.target_pos_z = target_pos_z
        self.target_pos_x = target_pos_x
        self.F_rep_z = 0
        self.F_rep_x = 0
        self.tmp_target_filter_z = LowPassFilter(0.1)
        self.tmp_target_filter_x = LowPassFilter(0.1)

    def evaluate(self, robot_pos_z: float, robot_pos_x: float, valid_points: Deque[Tuple[int, float, float, Tuple[int, int, int]]]) -> Tuple[float, float]:
        #calculate attractive forces
        F_att_z = self.k_att * (self.target_pos_z - robot_pos_z)
        F_att_x = self.k_att * (self.target_pos_x - robot_pos_x)

        #calculate repulsive forces
        self.F_rep_x = 0
        self.F_rep_z = 0

        for expire, obstacle_pos_x, obstacle_pos_z, color in valid_points:
            if color == (0, 0, 255):
                dist = math.hypot(robot_pos_x - obstacle_pos_x, robot_pos_z - obstacle_pos_z)
                if 0 < dist < self.rho_0:
                    self.F_rep_x += self.k_rep * (((1/dist) - (1/self.rho_0)) * (1/pow(dist, 3)) * (robot_pos_x - obstacle_pos_x))
                    self.F_rep_z += self.k_rep * (((1/dist) - (1/self.rho_0)) * (1/pow(dist, 3)) * (robot_pos_z - obstacle_pos_z))

        #potential field result
        robot_F_z = F_att_z + self.F_rep_z
        robot_F_x = F_att_x + self.F_rep_x

        #potential field target
        robot_target_pos_z = robot_pos_z + robot_F_z
        robot_target_pos_x = robot_pos_x + robot_F_x

        robot_target_pos_z = self.tmp_target_filter_z.evaluate(robot_target_pos_z)
        robot_target_pos_x = self.tmp_target_filter_x.evaluate(robot_target_pos_x)

        return robot_target_pos_z, robot_target_pos_x

    def set_target(self,target_pos_z: float, target_pos_x: float):
        self.target_pos_z = target_pos_z
        self.target_pos_x = target_pos_x


    def get_repulsive_force(self):
        return [self.F_rep_z,self.F_rep_x]
