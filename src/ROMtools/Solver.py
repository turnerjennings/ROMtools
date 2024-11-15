import numpy as np
from .RigidBody import *
from .SpringDamper import *
from .Configuration import *
from .Force import *
from time import time
import warnings
from typing import List


class Solver:
    def __init__(
        self,
        bodies: List[RigidBody],
        springs: List[SpringDamper],
        config: RunConfiguration,
        bcs: List[Union[Force, Displacement]] = None,
        verbose: bool = True,
    ) -> None:
        """Class to manage the solution process and store the subsequent output data

        Args:
            bodies (List[RigidBody]): List of bodies to include in the analysis
            springs (List[SpringDamper]): List of spring/dampers to include in the analysis
            config (RunConfiguration): RunConfiguration object which dictates run settings
            forces (List[Force], optional): List of external forces to include in the analysis. Defaults to None.
        """
        self.verbose = verbose
        self.solved = False
        self.config = config

        self.bodies = bodies
        self.n_bodies = len(bodies)

        self.springs = springs
        self.n_springs = len(springs)

        self.bcs = bcs

        self.solver = config.solver_type
        mass_temp = []
        self.constraint = []

        # check for contact
        self.contact = False
        for spring in springs:
            if spring.contact == True:
                self.contact = True

        for body in bodies:
            # generate mass vector for body
            mass_temp.append(body.m)
            mass_temp.append(body.m)
            mass_temp.append(body.I)
            self.constraint += list(body.constraint)

        self.m = np.array(mass_temp)

        self.timesteps = np.linspace(0.0, config.termination_time, config.n_timesteps)
        self.n_timesteps = config.n_timesteps
        self.dt = config.termination_time / config.n_timesteps

        # self.spring_force_array = np.empty((config.n_timesteps, self.n_springs*3))
        self.global_force_array = np.zeros((config.n_timesteps, self.n_bodies * 3))
        self.position_array = np.zeros((config.n_timesteps, self.n_bodies * 3))
        self.velocity_array = np.zeros((config.n_timesteps, self.n_bodies * 3))
        self.acceleration_array = np.zeros((config.n_timesteps, self.n_bodies * 3))

        # load initial conditions from each body
        for body in bodies:
            self.position_array[0, 3 * body.ID : 3 * body.ID + 3] = np.array(body.p0)
            self.velocity_array[0, 3 * body.ID : 3 * body.ID + 3] = np.array(body.v0)
            self.acceleration_array[0, 3 * body.ID : 3 * body.ID + 3] = np.array(
                body.a0
            )

    def Solve(self):
        """Generate a solution for the given inputs, does not return a value but populates the solver position, velocity, acceleration, and force arrays"""
        #set a flag for run failure
        fail = -1

        if self.verbose == True:
            print("Solving model...")
        start_time = time()

        for i in range(self.n_timesteps - 1):
            if self.config.solver_type == "RK4":
                self._RK4(i)
            elif self.config.solver_type == "FwdEuler":
                self._FwdEuler(i)
            elif self.config.solver_type == "CentralDifference":
                self._CentralDifference(i, self.config.alpha)
            else:
                raise TypeError("Solver type not supported")

            pcheck = (
                np.isnan(self.position_array[i, :])
                + np.isnan(self.velocity_array[i, :])
                + np.isnan(self.acceleration_array[i, :])
            )

            if np.sum(pcheck) > 0:
                fail = i
                warnings.warn(
                    f"WARNING: NaN values calculated at timestep {i}, terminating simulation"
                )
                np.nan_to_num(self.position_array, nan=0.0)
                np.nan_to_num(self.velocity_array, nan=0.0)
                np.nan_to_num(self.acceleration_array, nan=0.0)

                break

        # update spring force history
        for s in self.springs:
            if s.forcehist is not None:
                s.forcehist = np.array(s.forcehist)

        self._SaveData()
        end_time = time()
        if fail > 0 and self.verbose == True:
            print(
                f"Solution failed on timestep {i}, time elapsed: {end_time - start_time:.3f}"
            )
        elif self.verbose == True:
            print(f"Solution complete, time elapsed: {end_time - start_time:.3f}")
            self.solved = True
        else:
            self.solved = True

    def _SaveData(self):
        output_path = self.config.output_path + self.config.output_name

        header = ""
        for i in range(self.n_bodies):
            header = header + f"{i}-x,{i}-y,{i}-t"

        if self.config.output_position == True:
            if self.config.output_as_csv == True:
                np.savetxt(
                    output_path + "_position.csv",
                    self.position_array,
                    delimiter=",",
                    header=header,
                )
            else:
                np.save(output_path + "_position.npy", self.position_array)

        if self.config.output_velocity == True:
            if self.config.output_as_csv == True:
                np.savetxt(
                    output_path + "_velocity.csv",
                    self.velocity_array,
                    delimiter=",",
                    header=header,
                )
            else:
                np.save(output_path + "_velocity.npy", self.velocity_array)

        if self.config.output_acceleration == True:
            if self.config.output_as_csv == True:
                np.savetxt(
                    output_path + "_acceleration.csv",
                    self.acceleration_array,
                    delimiter=",",
                    header=header,
                )
            else:
                np.save(output_path + "_acceleration.npy", self.acceleration_array)

        if self.config.output_position == True:
            if self.config.output_as_csv == True:
                np.savetxt(
                    output_path + "_force.csv",
                    self.global_force_array,
                    delimiter=",",
                    header=header,
                )
            else:
                np.save(output_path + "_force.npy", self.global_force_array)

    def _SpringForces(
        self,
        p: np.ndarray,
        v: np.ndarray,
        t: float,
        t_idx: float,
        save_hist: bool = False,
    ) -> np.ndarray:
        forces_out = np.zeros(p.shape)

        for spring in self.springs:
            # update contact point
            if t_idx >= 1:
                if (
                    spring.contact == True
                    and spring.forcehist[t_idx - 1 : 0]
                    + spring.forcehist[t_idx - 1 : 1]
                    == 0
                ):
                    print("changing spring theta offset")
                    spring.t_offset = p[spring.parent + 2]

            spring_force = spring.calculate_force(p, v)

            forces_out += spring_force

            # update spring force history if requested
            if save_hist == True:
                spring.forcehist.append(
                    [
                        spring_force[3 * spring.parent],
                        spring_force[3 * spring.parent + 1],
                        spring_force[3 * spring.parent + 2],
                        spring_force[3 * spring.child + 2],
                    ]
                )
        return forces_out

    def _BodyForces(self, i: int):
        forces_out = np.zeros((self.n_bodies * 3))

        if self.bcs is not None:
            for f in self.bcs:
                if type(f) == Force:
                    forces_out[f.dof] = forces_out[f.dof] + f.ft[i]

        return forces_out

    # WIP prescribe displacement using penalty method.
    def _PrescribedDisp(self, i: int):
        pass

    def _RK4(self, i: int):
        h = self.dt

        p0 = self.position_array[i, :]
        v0 = self.velocity_array[i, :]

        F_next = self._SpringForces(p0, v0, self.timesteps[i], i, save_hist=True)

        F_next = F_next + self._BodyForces(i)

        self.global_force_array[i + 1, :] = F_next 

        k1 = np.divide(F_next, self.m)

        k2 = np.divide(
            self._SpringForces(
                p0 + 0.5 * h * v0 + (h**2) / 8 * k1,
                v0 + 0.5 * h * k1,
                self.timesteps[i] + 0.5 * h,
                i,
            ),
            self.m,
        )

        k3 = np.divide(
            self._SpringForces(
                p0 + 0.5 * h * v0 + (h**2) / 8 * k2,
                v0 + 0.5 * h * k2,
                self.timesteps[i] + 0.5 * h,
                i,
            ),
            self.m,
        )

        k4 = np.divide(
            self._SpringForces(
                p0 + h * v0 + (h**2) / 2 * k3, v0 + h * k3, self.timesteps[i] + h, i
            ),
            self.m,
        )

        self.position_array[i + 1, :] = (
            p0 + h * v0 + (h**2) / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        )
        self.position_array[i + 1, self.constraint] = self.position_array[
            0, self.constraint
        ]
        # self.position_array[i+1, :] = p0 + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        self.velocity_array[i + 1, :] = v0 + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        self.acceleration_array[i + 1, :] = k1

    def _FwdEuler(self, i: int):
        h = self.dt

        p0 = self.position_array[i, :]
        v0 = self.velocity_array[i, :]

        F_next = self._SpringForces(p0, v0, self.timesteps[i], i, save_hist=True)

        F_next = F_next + self._BodyForces(i)

        self.global_force_array[i + 1, :] = F_next
        
        self.acceleration_array[i + 1, :] = np.divide(
            self._SpringForces(p0, v0, self.timesteps[i], i), self.m
        )
        self.velocity_array[i + 1, :] = v0 + h * np.divide(
            self._SpringForces(p0, v0, self.timesteps[i], i), self.m
        )
        self.position_array[i + 1, :] = p0 + h * v0
        self.position_array[i + 1, self.constraint] = self.position_array[
            0, self.constraint
        ]

    def _CentralDifference(
        self, i: int, alpha: float, tolerance: float = 1e-2, maxiter: int = 100
    ):
        h = self.dt

        p0 = self.position_array[i, :]
        v0 = self.velocity_array[i, :]

        F_next = self._SpringForces(p0, v0, self.timesteps[i], i, save_hist=True)

        F_next = F_next + self._BodyForces(i)

        self.global_force_array[i + 1, :] = F_next

        self.acceleration_array[i + 1, :] = np.divide(
            self._SpringForces(p0, v0, self.timesteps[i], i), self.m
        )
        self.velocity_array[i + 1, :] = v0 + h * np.divide(
            self._SpringForces(p0, v0, self.timesteps[i], i), self.m
        )
        self.position_array[i + 1, :] = (
            p0
            + h * v0
            + 0.5
            * h**2
            * np.divide(self._SpringForces(p0, v0, self.timesteps[i], i), self.m)
        )
        self.position_array[i + 1, self.constraint] = self.position_array[
            0, self.constraint
        ]
