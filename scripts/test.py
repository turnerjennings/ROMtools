import os
import sys
from math import sqrt
from scipy.fft import fft, fftfreq,fftshift

src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))

sys.path.append(src_path)

from ROMtools import *

body1=Disc(0, 1, 1, cm=(0.0,1.0, 0.0),name="mass")
body1.initial_conditions(p=(0.0,0.1,0.0))

springstiffness = np.array([[0,0],[0.05,100],[0.1,10]])

spring1 = LinearSpringDamper(100,0,0,body1,child=None,child_pos=(0,0),type = "Tabular",curve=springstiffness)




config = RunConfiguration()
config.update_from_dict({
    "output_path":"T:/codes/ROMtools/scripts/",
    "output_name":"RK4",
    "n_timesteps": 10000,
    "termination_time":0.5

})

config_FE = RunConfiguration()
config_FE.update_from_dict({
    "output_path":"T:/codes/ROMtools/scripts/",
    "output_name":"fwdEuler",
    "solver_type": "FwdEuler",
    "n_timesteps": 10000,
    "termination_time":0.5
})

config_CD = RunConfiguration()
config_CD.update_from_dict({
    "output_path":"T:/codes/ROMtools/scripts/",
    "output_name":"CenDif",
    "solver_type": "CentralDifference",
    "n_timesteps": 10000,
    "termination_time":0.5
})

solver_RK = Solver([body1], [spring1], config=config)
solver_RK.Solve()

solver_FE = Solver([body1], [spring1], config=config_FE)
solver_FE.Solve()



solver_CD = Solver([body1], [spring1], config=config_CD)
solver_CD.Solve()



import matplotlib.pyplot as plt

validation = 0.1*np.cos(np.sqrt(100/1)*solver_RK.timesteps)+1

fig, ax = plt.subplots(1,1)

ax=plt.subplot(1,1,1)
ax.plot(solver_RK.timesteps,validation, label = "analytical")
ax.plot(solver_RK.timesteps,solver_RK.position_array[:,1], label = "RK4")
ax.plot(solver_FE.timesteps,solver_FE.position_array[:,1], label = "FwdEuler")
ax.plot(solver_CD.timesteps,solver_CD.position_array[:,1], label = "CenDif")
ax.legend()



plt.show()


