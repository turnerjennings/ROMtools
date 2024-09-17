import os
import sys
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt
from ROMtools import *

body1=Disc(ID=0, m=1,R=1,cm=(0,1,0))
body1.initial_conditions(p=(0.0,0.1,0.0))

fd_spring = np.array([[-0.1, -1],
                      [-0.07,0],
                      [0.07,0],
                      [0.1, 1]])

spring1 = LinearSpringDamper(1000,0,0,body1,child=None,child_pos=(0,0),type = "Tabular",curve=fd_spring)

config = RunConfiguration()
config.update_from_dict({
    "output_path":"T:/codes/ROMtools/scripts/",
    "output_name":"tabtest",
    "n_timesteps": 500,
    "termination_time":2

})

sol= Solver(bodies=[body1],springs=[spring1], config=config)
sol.Solve()
print(spring1.forcehist)

plt.plot(sol.position_array[1:,1],spring1.forcehist)
plt.show()

anim=Animator(sol)
anim.animate()