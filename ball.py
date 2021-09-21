import numpy as np
import math
from particle import Particle

class Ball:
    def __init__(self, particle_num, opp_robot_num):
        self.particle_num = particle_num
        self.opp_robot_num = opp_robot_num
        self.particles = []
        self.robot_particles_hit = []

    def initialize(self, loc, kick_dir, kick_speed, dir_sigma, dir_mu, speed_sigma, speed_mu):
        for j in range(self.opp_robot_num):
            self.robot_particles_hit.append([])

        for i in range(self.particle_num):
            theta = kick_dir
            speed = kick_speed
            vel = speed * np.array([math.cos(theta), math.sin(theta)])
            self.particles.append(Particle(loc, vel, np.array([0, 0])))
            for j in range(self.opp_robot_num):
                self.robot_particles_hit[j].append(False)

    def check_collision(self, robots):
        for i in range(self.particle_num):
            for ridx in range(len(robots)):
                for rpidx in range(self.particle_num):
                    if np.linalg.norm(self.particles[i].loc - robots[ridx].particles[rpidx].loc) < 0.1:
                        self.robot_particles_hit[ridx][rpidx] = True

    def step(self, dt):
        for i in range(self.particle_num):
            self.particles[i].step(dt)