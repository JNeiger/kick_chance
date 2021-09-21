import math
import numpy as np
from ball import Ball
from robot import Robot

particle_num = 1000

r1 = Robot(particle_num)
r2 = Robot(particle_num)
b = Ball(particle_num, 2)

r1.initialize(np.array([5, 5]), np.array([0, 0]), 0.01, 0)
r2.initialize(np.array([10, 10]), np.array([0, 0]), 0.01, 0)
b.initialize(np.array([4, 4]), 45 * math.pi/180, 0.5, 0.001, 0, 0.01, 0)

dt = 0.01
time_stop = 5
t = 0

while t < time_stop:

    b.check_collision([r1, r2])

    r1.step(dt)
    r2.step(dt)
    b.step(dt)
    t = t + dt
    print(t)