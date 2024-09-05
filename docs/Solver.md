# Solver

Class that stores and handles creation of a solution to a problem specified by a series of bodies, spring/dampers, and forces.

---

## Attributes

| Attribute |   Type   | Description|
|---------|-------------|-------------|
| solved | bool | Whether a solution to the given system has been found |
| config | RunConfiguration | Settings for solution |
| bodies | list [RigidBody] | Bodies included in the analysis |
| n_bodies | int | number of bodies in the analysis |
| springs | list [SpringDamper] | Spring/dampers included in the analysis |
| n_springs | int | number of spring/dampers in the analysis |
| bcs | list [Force, Displacement] | External boundary conditions included in the analysis |
| solver | "RK4", "FwdEuler", or "CentralDifference" | Time integration type for analysis |
| constraint | list[int] | list of constrained degrees of freedom |
| m | np.ndarray | 1d vector of mass/inertia for each body in the analysis |
| timesteps | np.ndarray | 1d vector of time points to evaluate |
| n_timesteps | int | number of timesteps in the analysis |
| dt | float | timestep for the analysis |
| global_force_array | np.ndarray | Array of size (n_timesteps, 3*n_bodies) storing force and torque vs. time applied to each degree of freedom |
| position_array | np.ndarray | Array of size (n_timesteps, 3*n_bodies) storing position vs. time for each degree of freedom |
| velocity_array | np.ndarray | Array of size (n_timesteps, 3*n_bodies) storing velocity vs. time for each degree of freedom |
| acceleration_array | np.ndarray | Array of size (n_timesteps, 3*n_bodies) storing acceleration vs. time for each degree of freedom |

---

# Methods

## ____init____

```python
class Solver:

    def __init__(
        self,
        bodies: List[RigidBody],
        springs: List[SpringDamper],
        config: RunConfiguration,
        bcs: List[Union[Force,Displacement]] = None,
        verbose: bool = False,
    ) -> None:



#create a rigid body
disc1 = Disc(ID=0,m=1.00,R=4.25,
            cm=(0,0,0),name="body1", 
            dof_constraint=(False, False, False))

#connect to boundary using a linear spring
spring = LinearSpringDamper(k=1000,c=10,
                            p=0.0,parent=disc1, child_pos = (0.0,0.0),type="Linear")

#create a run configuration
run_config=RunConfiguration()

#create a solver to perform analysis
sol = Solver([disc1],[spring],run_config)

```

| Attribute |   Type   | Description|
|---------|-------------|-------------|
| bodies | List [RigidBody] | Bodies to include in the analysis |
| springs | List [SpringDamper] | Spring/dampers to include in the analysis |
| config | RunConfiguration | Run settings for analysis |
| bcs | List [Force, Displacement] (optional) | External forces or prescribed motions for the analysis |
| verbose | bool | Specify whether to print solution steps and timing to the command window |

__Returns__

Solver object

## __Solve__

Calculate solution to defined problem

```python
#create a solver to perform analysis
sol = Solver([disc1],[spring],run_config)

#print solution x-position (no solution yet, all zeros)
print(sol.position_array[:,0])

#calculate solution to given problem
sol.solve()

#print solution x-position (solved, nonzero array)
print(sol.position_array[:,0])
```

__Returns__

None