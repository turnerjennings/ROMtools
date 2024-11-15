import numpy as np

from ROMtools import *

from time import time


body1 = Disc(ID=0, m=1, R=1, cm=(0, 1, 0))



config = RunConfiguration()
config.update_from_dict(
    {
        "output_path": "T:/codes/ROMtools/scripts/",
        "output_name": "forcetest",
        "n_timesteps": 250,
        "termination_time": 2,
        "solver_type": "RK4"
    }
)

def forcefun(t):
    return 10*np.sin(10*t)

f = Force(config=config, f=forcefun, body=0, dof=1)

print(f.ft)
solver_RK = Solver(bodies=[body1], springs=[], bcs=[f], config=config)

solver_RK.Solve()

# anim = Animator(solver_RK)
# anim.animate()


import matplotlib.pyplot as plt

fig, ax = plt.subplots(1, 1)

ax = plt.subplot(1, 1, 1)
ax.plot(solver_RK.timesteps, solver_RK.position_array[:,1], label="true")
ax.legend()


plt.show()
