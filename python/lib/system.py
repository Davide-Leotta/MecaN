from typing import Tuple
from numpy.typing import NDArray
import numpy as np
import math

class Proportional:
    def __init__(self, k: float):
        self.k = k

    def evaluate(self, delta_t: float, u: float) -> float:
        return u * self.k

class Integrator:
    def __init__(self, ki: float):
        self.acc = 0
        self.ki = ki

    def evaluate(self, delta_t: float, u: float) -> float:
        out = self.acc + u * delta_t
        self.acc = out
        out *= self.ki
        return out

class Derivator:
    def __init__(self,kd: float):
        self.prev = 0
        self.kd = kd

    def evaluate(self, delta_t: float, u: float) -> float:
        out = (u - self.prev) / delta_t
        self.prev = u
        out *= self.kd
        return out

def saturate(inp: float, sat: float) -> Tuple[float,bool]:
    if inp > sat:
        return (sat, True)
    elif inp < -sat:
        return (-sat, True)
    return (inp, False)

class PID:
    def __init__(self, k: float , ki: float, kd: float, sat: float):
        self.P = Proportional(k)
        self.I = Integrator(ki)
        self.D = Derivator(kd)
        self.sat = sat
        self.in_sat = False

    def evaluate(self, delta_t: float, u: float) -> float:
        out = self.P.evaluate(delta_t, u)

        if self.in_sat:
            out += self.I.acc * self.I.ki

        else:
            out += self.I.evaluate(delta_t, u)

        out += self.D.evaluate(delta_t, u)

        out, self.in_sat = saturate(out, self.sat)

        return out

def inv_kin(v: NDArray[float], r: float, l: float) -> NDArray[float]:
    k_rot = 0.6
    return  (1/r) * np.dot([[-1, 1, -k_rot * l], [1, 1, -k_rot * l], [1, -1, -k_rot * l], [-1, -1, -k_rot * l]], v)

class MecanumController:
    def __init__(self, k: float, ki: float, kd: float, sat: float, R: float, L: float):
        self.PID_w1 = PID(k, ki, kd, sat) 
        self.PID_w2 = PID(k, ki, kd, sat)
        self.PID_w3 = PID(k, ki, kd, sat)
        self.PID_w4 = PID(k, ki, kd, sat)
        self.R = R
        self.L = L

    def evaluate(self, delta_t: float, u: float) -> NDArray[float]:
        err = inv_kin(u, self.R, self.L)
        w1 = self.PID_w1.evaluate(delta_t, err[0])
        w2 = self.PID_w2.evaluate(delta_t, err[1])
        w3 = self.PID_w3.evaluate(delta_t, err[2])
        w4 = self.PID_w4.evaluate(delta_t, err[3])
        return [w1, w2, w3, w4]

class VirtualRobot:
    ACC = 0
    CRUISE = 1
    DEC = 2
    TARGET = 3

    def __init__(self,p_target: float,acc: float,v_max: float,dec: float):
        self.dir = 1 if p_target >= 0 else -1
        self.p_target = abs(p_target)
        self.acc = abs(acc)
        self.v_max = abs(v_max)
        self.dec = abs(dec)

        self.v = 0
        self.p = 0
        #self.t_dec = self.t_acc + (self.p_target/self.v_max) - (self.v_max /(2 * self.acc)) - (self.v_max)/(2 * self.dec)
        self.phase = VirtualRobot.ACC

    def evaluate(self,delta_t: float) -> float:
        match self.phase:
            case VirtualRobot.ACC:
                self.p = self.p + self.v * delta_t + ((1/2) * self.acc * delta_t * delta_t )
                self.v = self.v + self.acc * delta_t
                if self.v >= self.v_max:
                    self.v = self.v_max
                    self.phase = VirtualRobot.CRUISE
                if (self.p + (self.v * self.v) / ( 2 * self.dec)) >= self.p_target:
                    self.phase = VirtualRobot.DEC
            case VirtualRobot.CRUISE:
                self.p = self.p + self.v * delta_t
                if (self.p + (self.v * self.v) / ( 2 * self.dec)) >= self.p_target:
                    self.phase = VirtualRobot.DEC
            case VirtualRobot.DEC:
                self.p = self.p + self.v * delta_t - ((1/2) * self.dec * delta_t * delta_t)
                self.v = self.v - self.dec * delta_t
                if self.v < 0:
                    self.v = 0
                    self.p = self.p_target
                    self.phase = VirtualRobot.TARGET
            case VirtualRobot.TARGET:
                self.v = 0
        return self.v * self.dir

class PositionController:
    def __init__(self, k: float, ki: float, kd: float, v_cruise: float):
        self.Pz = PID(k, ki, kd, v_cruise)
        self.Px = PID(k, ki, kd, v_cruise)

    def evaluate(self, delta_t: float, u: NDArray[float]) -> NDArray[float]:
        vz = self.Pz.evaluate(delta_t, u[0])
        vx = self.Px.evaluate(delta_t, u[1])
        vel = np.array([vz, vx])
        return vel

class OrientationController:
    def __init__(self, k: float, ki: float, kd: float, w_max: float):
        self.PID = PID(k, ki, kd, w_max)

    def evaluate(self, delta_t: float, u: float) -> float:
        w = self.PID.evaluate(delta_t, u)

        return w

def global_to_local(ang: float, global_coordinates: NDArray[float]) -> NDArray[float]:
    rad = math.radians(ang)
    rot_matrix = np.array([
        [math.cos(rad), math.sin(rad)],
        [-math.sin(rad), math.cos(rad)]
    ])
    return rot_matrix.dot(global_coordinates)

class LowPassFilter:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.prev = None

    def evaluate(self, u: float) -> float:
        if self.prev is None:
            self.prev = u
            return u

        out = self.alpha * u + (1 - self.alpha) * self.prev
        self.prev = out
        return out