import numpy as np
from typing import Literal, Callable, List, Union
from .Configuration import RunConfiguration


class BoundaryCondition:
    def __init__(
        self,
        config: RunConfiguration,
        f: Union[float, np.ndarray, Callable[[np.ndarray], np.ndarray]],
        body: Union[List[int], int],
        dof: Literal[0, 1, 2],
        *args,
        **kwargs,
    ) -> None:
        """Class to store a external force to be applied to a solver system.  Force can be provided as a function, numpy array, or scalar value.

        Args:
            config (RunConfiguration): Run configuration information for timestep controls
            f (Union[float, np.ndarray, Callable[[np.ndarray],np.ndarray]]): Force input, can be a function which takes an argument t and optional additional arguments, a numpy array, or a scalar value
            body (Union[List[int], int]): ID of the body or bodies to apply the load to
            dof (Literal[0,1,2]): degree of freedom (x, y, or theta) to apply the load on
        """

        self.bodies = np.array(body)
        self.dof = 3 * self.bodies + dof
        self.t = np.linspace(0, config.termination_time, config.n_timesteps)


class Force(BoundaryCondition):
    def __init__(
        self,
        config: RunConfiguration,
        f: Union[float, np.ndarray, Callable[[np.ndarray], np.ndarray]],
        body: Union[List[int], int],
        dof: Literal[0, 1, 2],
        *args,
        **kwargs,
    ) -> None:
        super().__init__(config, f, body, dof, *args, **kwargs)

        if type(f) == np.ndarray:
            if f.shape[0] < self.t.shape[0]:
                pad = self.t.shape[0] - f.shape[0]
                self.ft = np.pad(f, (0, pad), mode="constant", constant_values=0.0)
            elif f.shape[0] > self.t.shape[0]:
                self.ft = f[: self.t.shape[0]]
            else:
                self.ft = f
        elif type(f) == float:
            self.ft = f * np.ones(self.t.shape)
        else:
            self.ft = f(self.t, *args, **kwargs)


class Displacement(BoundaryCondition):
    def __init__(
        self,
        config: RunConfiguration,
        f: Union[float, np.ndarray, Callable[[np.ndarray], np.ndarray]],
        body: Union[List[int], int],
        dof: Literal[0, 1, 2],
        *args,
        **kwargs,
    ) -> None:
        super().__init__(config, f, body, dof, *args, **kwargs)

        if type(f) == np.ndarray:
            if f.shape[0] < self.t.shape[0]:
                pad = self.t.shape[0] - f.shape[0]
                self.dt = np.pad(f, (0, pad), mode="constant", constant_values=0.0)
            elif f.shape[0] > self.t.shape[0]:
                self.dt = f[: self.t.shape[0]]
            else:
                self.dt = f
        elif type(f) == float:
            self.dt = f * np.ones(self.t.shape)
        else:
            self.dt = f(self.t, *args, **kwargs)
