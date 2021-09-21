import numpy as np

class Particle:
    def __init__(self, loc, vel, accel):
        self.loc = loc
        self.vel = vel
        self.accel = accel

    def step(self, dt):
        self.loc = self.loc + np.multiply(dt, self.vel)
        self.vel = self.vel + np.multiply(dt, self.accel)