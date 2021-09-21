import numpy as np
from particle import Particle

class Robot:
    def __init__(self, particle_num):
        self.particle_num = particle_num
        self.particles = []

    def initialize(self, loc, vel, sigma, mu):
        for i in range(self.particle_num):
            self.particles.append(Particle(loc, vel, np.array([0, 0])))

    def step(self, dt):
        for i in range(self.particle_num):
            self.particles[i].step(dt)