from lib.data.dataplot import *
from lib.utils.time import *
from lib.dds.dds import *

class Proportional:

    def __init__(self, k):
        self.k = k

    def evaluate(self, delta_t, u):
        return u * self.k

class Integrator:

    def __init__(self):
        self.u = 0

    def evaluate(self, delta_t, u):
        u = self.u + u * delta_t
        self.u = u
        return u

class Control:

    def __init__(self, kp, ki, sat):
        self.P = Proportional(kp)
        self.I = Integrator()
        self.ki = ki
        self.target = 0.0
        self.sat = sat

    def set_target(self,target):
        self.target = target

    def evaluate(self, delta_t, u):
        err = self.target - u
        p_out = self.P.evaluate(delta_t,err)
        i_out = self.I.evaluate(delta_t,p_out)
        if i_out > self.sat:
            i_out = self.sat
        return i_out

dds = DDS()
dds.start()
dds.subscribe(['position'])

s = Control(1,1,100)
s.set_target(500.0)

t = Time()
t.start()

while t.get() < 5:
    p = dds.wait('position')
    print(p)
    delta_t = t.elapsed()
    output = s.evaluate(delta_t,p)
    dds.publish('force',output,dds.DDS_TYPE_FLOAT)
