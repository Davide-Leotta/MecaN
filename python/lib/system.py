import numpy as np

class Proportional:
    def __init__(self, k):
        self.k = k

    def evaluate(self, delta_t, u):
        return u * self.k

class Integrator:
    def __init__(self, ki):
        self.acc = 0
        self.ki = ki

    def evaluate(self, delta_t, u):
        out = self.acc + u * delta_t
        self.acc = out
        out *= self.ki
        return out

def saturate(inp, sat):
    if inp > sat:
        return (sat, True)
    elif inp < -sat:
        return (-sat, True)
    return (inp, False)

class PI:
    def __init__(self, k , ki, sat):
        self.P = Proportional(k)
        self.I = Integrator(ki)
        self.sat = sat
        self.in_sat = False

    def evaluate(self, delta_t, u):
        out = self.P.evaluate(delta_t, u)

        if self.in_sat:
            out += self.I.acc * self.I.ki

        else:
            out += self.I.evaluate(delta_t, u)

        out, self.in_sat = saturate(out, self.sat)

        return out

def inv_kin(v, r, l):
    k_rot = 0.6
    return  (1/r) * np.dot([[-1, 1, -k_rot * l], [1, 1, -k_rot * l], [1, -1, -k_rot * l], [-1, -1, -k_rot * l]], v)

class MecanumController:
    def __init__(self, k, ki, sat, R, L):
        self.PI_w1 = PI(k, ki, sat) 
        self.PI_w2 = PI(k, ki, sat)
        self.PI_w3 = PI(k, ki, sat)
        self.PI_w4 = PI(k, ki, sat)
        self.R = R
        self.L = L
        self.target = [0, 0, 0]

    def set_target(self, v):
        self.target = np.array(v)

    def evaluate(self, delta_t, u):
        v  = np.round(self.target - u, 3)
        err = inv_kin(v, self.R, self.L)
        err = np.round(err, 2)
        w1 = self.PI_w1.evaluate(delta_t, err[0])
        w2 = self.PI_w2.evaluate(delta_t, err[1])
        w3 = self.PI_w3.evaluate(delta_t, err[2])
        w4 = self.PI_w4.evaluate(delta_t, err[3])
        return [w1, w2, w3, w4]

class VirtualRobot:
    ACC = 0
    CRUISE = 1
    DEC = 2
    TARGET = 3
    
    def __init__(self,p_target,acc,v_max,dec):
        self.dir = 1 if p_target >= 0 else -1
        self.p_target = abs(p_target)
        self.acc = abs(acc)
        self.v_max = abs(v_max)
        self.dec = abs(dec)

        self.v = 0
        self.p = 0
        #self.t_dec = self.t_acc + (self.p_target/self.v_max) - (self.v_max /(2 * self.acc)) - (self.v_max)/(2 * self.dec)
        self.phase = VirtualRobot.ACC

    def evaluate(self,delta_t):
        match self.phase:
            case VirtualRobot.ACC:
                self.p = self.p + self.v * delta_t + ((1/2) * self.acc * delta_t * delta_t )
                self.v = self.v + self.acc * delta_t
                if self.v >= self.v_max:
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
                    self.phase = VirtualRobot.TARGET
            case VirtualRobot.TARGET:
                self.v = 0
        return self.v * self.dir

class PositionController:
    def __init__(self, kz, kx, kt):
        self.Pz = Proportional(kz)
        self.Px = Proportional(kx)
        self.Pt = Proportional(kt)

    def evaluate(self,delta_t,u):
        vz = self.Pz.evaluate(delta_t,u[0])
        vx = self.Px.evaluate(delta_t,u[1])
        vt = self.Pt.evaluate(delta_t,u[2])
        return np.array([vz,vx,vt])