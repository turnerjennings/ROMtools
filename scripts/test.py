import os
import sys
from math import sqrt
from scipy.fft import fft, fftfreq,fftshift

src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))

sys.path.append(src_path)

from rom_rbd import *

body1=Disc(0, 1, 1, cm=(0.0,1.0, 0.0),name="body1")
body1.initial_conditions(p=(0.0,0.0,0.0))

body2 = Disc(1, 2, 1, cm = (1.0, 0.0, 0.0), name="body2")

spring1 = RotationalSpringDamper(100,1,body1)
spring2 = LinearSpringDamper(100,1,body1)




config = RunConfiguration()
config.update_from_dict({
    "output_name":"RK4",
    "n_timesteps": 10000,
    "termination_time":2
})

def sinforce(t:np.ndarray,a:float,w:float)->np.ndarray:
    return a*np.sin(w*t)

f1 = Force(config, sinforce, 0, 1,a=1.0,w=10.0)
solver_RK = Solver([body1], [spring2], config=config, forces=[f1])

solver_RK.Solve()

anim = Animator(solver_RK)
anim.animate()

import matplotlib.pyplot as plt

validation = 0.1*np.cos(np.sqrt(100/1)*solver_RK.timesteps)+1

fig, ax = plt.subplots(2,1)

ax[0]=plt.subplot(2,1,1)
ax[0].plot(solver_RK.timesteps,validation, label = "analytical")
ax[0].plot(solver_RK.timesteps,solver_RK.position_array[:,1], label = "RK4")
ax[0].legend()

fft1 = fftshift(fft(solver_RK.position_array[:,1]))
fft2 = fftshift(fft(validation))

freq = fftshift(fftfreq(solver_RK.n_timesteps))/solver_RK.dt

ax[1]=plt.subplot(2,1,2)
ax[1].plot(freq, fft2, label = "analytical")
ax[1].plot(freq, fft1, label = "RK4")
ax[1].legend()
ax[1].set_xlim(0,12)

#plt.show()


