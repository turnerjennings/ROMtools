# Boundary Conditions

External boundary conditions can be applied to the model in the form of forces or prescribed displacements (__WIP__).

---

# Forces

External forces can be applied to a body along one its degrees of freedom.  Forces can be specified using a function, numpy vector, or scalar. 

## Attributes

| Attribute |   Type   | Description|
|---------|-------------|-------------|
| bodies | int or list[int] | Body or list of bodies that the force acts on |
| dof | 0, 1, or 2 | Degree of freedom the force acts on (1 = x, 2 = y, 3 = theta) |
| t | np.ndarray | array of timesteps for the simulation |
| ft | np.ndarray | value of the force function evaluated at each timestep |

# Methods

## ____init____

```python
class Force(BoundaryCondition):
    
    def __init__(self,
                 config: RunConfiguration, 
                 f: Union[float, np.ndarray, Callable[[np.ndarray], np.ndarray]], 
                 body: Union[List[int], int], 
                 dof: Literal[0, 1, 2], 
                 *args, 
                 **kwargs) -> None:

#Define a constant force of -1g in the y-direction
fgrav = Force(config, -9.81, body=1, dof=1)

#Define a sinusoidal force in the x-direction
a=5
def sinforce(t,a):
    return a*np.sin(t)
fsinusoid = Force(config, sinforce, 0, 0, a)

```
__inputs__

| Attribute |   Type   | Description|
|---------|-------------|-------------|
| Config | RunConfiguration | Run configuration for the simulation |
| f | float or np.ndarray or function | force definition |
| body | int or list [int] | body or list of bodies that the force acts on|
| dof | 0 or 1 or 2 | Degree of freedom the force acts on (1 = x, 2 = y, 3 = theta) |
| args, kwargs | | Additional arguments required for f if f is a function |

__returns__

Force object

__notes__

If $f$ is a scalar, ft is a consistent numpy array of that scalar.  If $f$ is a numpy array, ft is that array with zero padding on the end to achieve the same length as the number of timesteps in the configuration file. If $f$ is a function, it must accept an argument t, and ft will be the value of that function evaluated at each timestep in the numpy array t.

---

# Displacement

__WIP__