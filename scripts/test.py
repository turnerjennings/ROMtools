import numpy as np

from ROMtools import *

body1 = Disc(ID=0, m=1, R=1, cm=(0, 1, 0))
body1.initial_conditions(p=(0.0, 0.1, 0.0))

spring1 = LinearSpringDamper(
    100, 0, 0, body1, child=None, child_pos=(0, 0), type="Linear"
)


config = RunConfiguration()
config.update_from_dict(
    {
        "output_path": "T:/codes/ROMtools/scripts/",
        "output_name": "RK4",
        "n_timesteps": 250,
        "termination_time": 2,
    }
)

solver_RK = Solver(bodies=[body1], springs=[spring1], config=config)
solver_RK.Solve()

# anim = Animator(solver_RK)
# anim.animate()


import matplotlib.pyplot as plt

validation = 0.1 * np.cos(np.sqrt(100 / 1) * solver_RK.timesteps) + 1

color = (solver_RK.position_array[:, 1] - validation) ** 2

fig, ax = plt.subplots(1, 1)

ax = plt.subplot(1, 1, 1)
ax.plot(solver_RK.timesteps, validation, label="analytical")
ax.scatter(
    solver_RK.timesteps,
    solver_RK.position_array[:, 1],
    label="RK4",
    c=color,
    cmap="magma",
)
ax.legend()


plt.show()
