import numpy as np
from .RigidBody import *
from .SpringDamper import *
from .Configuration import *
from .Force import Force
from time import time
import warnings

class Solver():

    def __init__(self, bodies:list[RigidBody],springs:list[SpringDamper], config:RunConfiguration, forces:list[Force] = None) -> None:
        self.config = config
        
        self.bodies = bodies
        self.n_bodies = len(bodies)

        self.springs = springs
        self.n_springs = (len(springs))

        self.forces = forces

        self.solver = config.solver_type
        mass_temp = []
        for body in bodies:
            mass_temp.append(body.m)
            mass_temp.append(body.m)
            mass_temp.append(body.I)


        self.constraint=[]

        for body in self.bodies:
            self.constraint += list(body.constraint)


        self.m=np.array(mass_temp)

        self.timesteps = np.linspace(0.0,config.termination_time,config.n_timesteps)
        self.n_timesteps = config.n_timesteps
        self.dt = config.termination_time/config.n_timesteps

        #self.spring_force_array = np.empty((config.n_timesteps, self.n_springs*3))
        self.global_force_array = np.zeros((config.n_timesteps, self.n_bodies*3))
        self.position_array = np.zeros((config.n_timesteps, self.n_bodies*3))
        self.velocity_array = np.zeros((config.n_timesteps, self.n_bodies*3))
        self.acceleration_array = np.zeros((config.n_timesteps, self.n_bodies*3))
        

        #load initial conditions from each body
        for body in bodies:
            self.position_array[0,3*body.ID:3*body.ID+3] = np.array(body.p0)
            self.velocity_array[0,3*body.ID:3*body.ID+3] = np.array(body.v0)
            self.acceleration_array[0,3*body.ID:3*body.ID+3] = np.array(body.a0)

    def Solve(self):
        fail = -1
        print("Solving model...")
        start_time = time()
        for i in range(self.n_timesteps-1):
            if self.config.solver_type == "RK4":
                    self._RK4(i)
            elif self.config.solver_type == "FwdEuler":
                    self._FwdEuler(i)
            elif self.config.solver_type == "CentralDifference":
                    self._CentralDifference(i,self.config.alpha)
            else:
                raise TypeError("Solver type not supported")
            
            pcheck = np.isnan(self.position_array[i,:])+np.isnan(self.velocity_array[i,:])+np.isnan(self.acceleration_array[i,:])
            
            if np.sum(pcheck) > 0:
                fail = i
                warnings.warn("WARNING: NaN values calculated at timestep {i}, terminating simulation")
                np.nan_to_num(self.position_array, nan=0.0)
                np.nan_to_num(self.velocity_array, nan=0.0)
                np.nan_to_num(self.acceleration_array, nan=0.0)

                break

        self._SaveData()
        end_time = time()
        if fail > 0:
            print(f"Solution failed on timestep {i}, time elapsed: {end_time - start_time:.3f}")
        else:
            print(f"Solution complete, time elapsed: {end_time - start_time:.3f}")



    def _SaveData(self):
        output_path = self.config.output_path + self.config.output_name

        header = ""
        for i in range(self.n_bodies):
            header = header + f"{i}-x,{i}-y,{i}-t"

        if self.config.output_position == True:
            if self.config.output_as_csv == True:
                np.savetxt(output_path + "_position.csv", self.position_array,delimiter=',',header=header)
            else:
                np.save(output_path + "_position.npy", self.position_array)

        if self.config.output_velocity == True:
            if self.config.output_as_csv == True:
                np.savetxt(output_path + "_velocity.csv", self.velocity_array,delimiter=',',header=header)
            else:
                np.save(output_path + "_velocity.npy", self.velocity_array)

        if self.config.output_acceleration == True:
            if self.config.output_as_csv == True:
                np.savetxt(output_path + "_acceleration.csv", self.acceleration_array,delimiter=',',header=header)
            else:
                np.save(output_path + "_acceleration.npy", self.acceleration_array)

        if self.config.output_position == True:
            if self.config.output_as_csv == True:
                np.savetxt(output_path + "_force.csv", self.global_force_array,delimiter=',',header=header)
            else:
                np.save(output_path + "_force.npy", self.global_force_array)   
            
    
    def _SpringForces(self, p:np.ndarray, v:np.ndarray, t:float, t_idx:float) -> np.ndarray:
        forces_out = np.empty(p.shape)
        for spring in self.springs:
            forces_out += spring.calculate_force(p,v)

        return forces_out    

    def _BodyForces(self, i:int):
        forces_out = np.zeros((self.n_bodies*3)) 

        if self.forces is not None:
            for f in self.forces:
                forces_out[f.dof] = forces_out[f.dof] + f.ft[i]

        return forces_out
        
        

    def _RK4(self, i:int):
        h=self.dt

        p0 = self.position_array[i,:]
        v0 = self.velocity_array[i,:]

        self.global_force_array[i+1, :] = self._SpringForces(p0,v0,self.timesteps[i], i) + self._BodyForces(i)

        k1 = np.divide(self._SpringForces(p0,v0,self.timesteps[i], i),self.m)
        k2 = np.divide(self._SpringForces(p0+0.5*h*v0+(h**2)/8*k1, v0+0.5*h*k1, self.timesteps[i]+0.5*h, i), self.m)
        k3 = np.divide(self._SpringForces(p0+0.5*h*v0+(h**2)/8*k2, v0+0.5*h*k2, self.timesteps[i]+0.5*h, i), self.m)
        k4 = np.divide(self._SpringForces(p0+h*v0+(h**2)/2*k3, v0+h*k3, self.timesteps[i]+h, i), self.m)

        self.position_array[i+1, :] = p0 + h*v0 + (h**2)/6*(k1+2*k2+2*k3 + k4)
        self.position_array[i+1, self.constraint] = self.position_array[0, self.constraint]
        #self.position_array[i+1, :] = p0 + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        self.velocity_array[i+1, :] = v0 + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        self.acceleration_array[i+1, :] = k1    

    
    def _FwdEuler(self, i:int):
        h=self.dt

        p0 = self.position_array[i,:]
        v0 = self.velocity_array[i,:]

        self.global_force_array[i+1, :] = self._SpringForces(p0,v0,self.timesteps[i], i)
        self.acceleration_array[i+1, :] = np.divide(self._SpringForces(p0,v0,self.timesteps[i], i),self.m)
        self.velocity_array[i+1, :] = v0 + h*np.divide(self._SpringForces(p0,v0,self.timesteps[i], i),self.m)
        self.position_array[i+1, :] = p0 + h*v0 + 0.5*h**2*np.divide(self._SpringForces(p0,v0,self.timesteps[i], i),self.m)
        self.position_array[i+1, self.constraint] = self.position_array[0, self.constraint]

    
    def _CentralDifference(self,i:int, alpha:float, tolerance:float = 1e-2, maxiter:int = 100):
        h=self.dt

        print(f"timestep {i}")

        p0 = self.position_array[i,:]
        v0 = self.velocity_array[i,:]
        a0 = self.acceleration_array[i,:]

        if i == 0:
            p_minus_one = p0-h*v0 - 0.5*h**2*a0
        else:
            p_minus_one = self.position_array[i-1,:]

        tol = 1
        iter = 0

        while tol > tolerance:

            self.position_array[i+1,:] = ((h**2)/self.m)*self._SpringForces(p0,v0,self.timesteps[i],i) + 2*p0 - p_minus_one
            self.position_array[i+1, self.constraint] = self.position_array[0, self.constraint]
            vguess = 1/(2*h) * (self.position_array[i+1] - p_minus_one)
            v0 = v0 + alpha * vguess
            tol = np.sum(v0-vguess)**2

            print(f"Convergence error: {tol}")

            iter += 1
            if iter >= maxiter:
                raise TimeoutError(f"Convergence not reached on timestep {i}...")










