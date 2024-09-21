import math
from math import pi, cos, sin
import matplotlib.pyplot as plt
from ROMtools import *

config_dict = {
    "output_path": "",
    "output_name": "coupletest",
    "solver_type": "RK4",
    "termination_time": 0.05,
    "n_timesteps": 500,
    "animate_concise": True,
}
# define bodies
run_config = RunConfiguration()
run_config.update_from_dict(config_dict=config_dict)

R_head = 0.099
head = Disc(
    0,
    4.66,
    R_head,
    I=0.0229,
    cm=(0, 0, 0),
    name="head",
    dof_constraint=(False, False, False),
)

R_helmet = 0.1176
helmet_cm = (0, 4 * R_helmet / (3 * pi), 0)
# helmet_cm = (0,0,0)
helmet = Semicircle(
    1, 1.01, R_helmet, name="helmet", cm=helmet_cm, dof_constraint=(False, False, False)
)
helmet.initial_conditions(p=(-0.005, 0, 0), v=(0, 1, 0))

# define pad positions
pad_angles = [0]
pad_helmet_pos = [
    (
        R_helmet * cos(math.radians(i)),
        R_helmet * sin(math.radians(i)) - 4 * R_helmet / (3 * pi),
    )
    for i in pad_angles
]
# pad_head_pos = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]
pad_head_pos = (0, 0.000)


pad1linear = LinearSpringDamper(
    8000,
    0.0,
    p=-0.001,
    parent=helmet,
    child=head,
    parent_pos=pad_helmet_pos[0],
    child_pos=pad_head_pos,
    type="Linear",
    mu=(10, 0),
)


solver = Solver([head, helmet], [pad1linear], run_config)
# pad4linear, pad5linear, pad6linear,
solver.Solve()

anim = Animator(solver)
anim.animate()

fig = plt.figure()
plt.plot(solver.timesteps, solver.global_force_array[:, 2], label="total y-force")
plt.plot(solver.timesteps[1:], pad1linear.forcehist[:, 2], label="spring y-force")
plt.legend()

plt.show()
