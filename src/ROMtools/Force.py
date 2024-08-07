import numpy as np
from typing import Literal, Callable
from.Configuration import RunConfiguration

class Force:

    def __init__(self, config:RunConfiguration, f:float | np.ndarray | Callable[[np.ndarray],np.ndarray], body:list[int] | int, dof:Literal[0,1,2], *args, **kwargs) -> None:
        self.bodies = np.array(body)
        self.dof = 3*self.bodies + dof
        self.t=np.linspace(0,config.termination_time,config.n_timesteps)

        if type(f) == np.ndarray:
            if f.shape[0] < self.t.shape[0]:
                pad = self.t.shape[0]-f.shape[0]
                self.ft = np.pad(f,(0,pad), mode='constant',constant_values=0.0)
            elif f.shape[0] > self.t.shape[0]:
                self.ft = f[:self.t.shape[0]]
            else:
                self.ft = f
        elif type(f) == float:
            self.ft = f*np.ones(self.t.shape)
        else:
            self.ft = f(self.t, *args, **kwargs)