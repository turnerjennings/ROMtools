# ROMtools

Functions for reduced order rigid body modelling of dynamical systems

## Description

ROMtools is a workflow for defining 2-D rigid body dynamics simulations.  The library was developed for the purposes of creating reduced order physics models of complex deformable systems. ROMtools integrates efficiently with existing convex optimization libraries (e.g., scipy.optimize) for inverse parameter fitting to higher-order models or for design optimization.  


## Installation

ROMtools can be installed via pip:

```
py -m pip install ROMtools
```

## Features

A full description of all features can be found in the [ROMtools documentation](https://turnerjennings.github.io/ROMtools/).  In short, a ROMtools model consists of the following components

- Rigid bodies defined by their inertia and geometry
- Spring/dampers connecting rigid bodies to each other and to boundaries
- (optional) external forces
- A RunConfiguration object which stores preferences about solution methods and desired outputs
- A solver object which calculates and stores the results of the simulation
- (optional) Animator to plot solver results

(Somewhat) unique features of ROMtools include:

- Object-oriented workflow which plays well with integration into larger analyses
- Non-linear spring models
- O(t<sup>5</sup>) accurate Runge-Kutta-Nyström time integration

## Examples

### 1-DOF vibration

Below is an example workflow for defining a simple 1-degree-of-freedom vibration problem, calculating the solution, and creating an animation of the result.
```python
import ROMtools as ROM
import numpy as np

#define a rigid disc body at (0,1) constrained to move in y-direction
rigid_body = ROM.Disc(
                      ID = 0, 
                      m = 1, 
                      R = 1, 
                      name = "mass", 
                      cm = (0.0, 1.0, 0.0),
                      dof_constraint = (True, False, True)
                      )

#define an initial displacement and initial velocity for the disc
rigid_body.initial_conditions(
                              p = (0.0, 0.1, 0.0),
                              v = (0.0, 1.0, 0.0)
                              )

#define a spring and damper connecting the disc to the origin
spring = ROM.LinearSpringDamper(
                            k=100,c=0,
                            parent=body1,child=None, 
                            child_pos=(0.0,0.0),
                            type = "Linear"
                            )

#define a configuration file to handle run settings
run_config = ROM.RunConfiguration()
run_config.update_from_dict({
    "output_path":"/path/to/dir/",
    "output_name":"1DOF vibration",
    "n_timesteps": 1000,
    "termination_time": 0.5,
    "solver_type": "RK4"
})

#define a solver to store solution and run simulation
soln = ROM.Solver(
                  bodies = [rigid_body],
                  springs = [spring],
                  config = run_config
                  )

soln.Solve()

#create an animation of the solution
anim = ROM.Animator(soln)
anim.animate()
```

### Optimization with Scipy Optimize

Below is an example of fitting the above 1DOF model to generated data, using scipy optimize and treating the spring stiffness as the free parameter:

```python
import ROMtools as ROM
import numpy as np
import scipy.optimize as spo

#create sinusoidal "experimental data" to fit model to
t = np.linspace(0,1,1000)
a=100
b=0.1
c=1.0
x_compare = b*np.sin(a*t)+c

#define function to return the residual between 1dof model and comparison data
def 1dof_minimize(x, x_compare):
    #input x parameters: x = spring k

    #system rigid body
    rigid_body = ROM.Disc(0, 1, 1, name = "mass", 
                        cm = (0.0, 1.0, 0.0),
                        dof_constraint = (True, False, True)
                        )

    rigid_body.initial_conditions(p = (0.0, 0.1, 0.0))
    #define spring with x as free parameter
    spring = ROM.LinearSpringDamper(
                                k=x,c=0,
                                parent=body1,child=None, 
                                child_pos=(0.0,0.0),
                                type = "Linear"
                                )

    #define run configuration with no file outputs
    run_config = ROM.RunConfiguration()
    run_config.update_from_dict({
        "output_path":"",
        "output_name":"1DOF vibration",
        "n_timesteps": 1000,
        "termination_time": 1,
        "output_position": False,
        "output_velocity": False,
        "output_acceleration": False,
        "output_force": False
    })

    #define a solver to store solution and run simulation
    soln = ROM.Solver(
                    bodies = [rigid_body],
                    springs = [spring],
                    config = run_config
                    )

    soln.Solve()
    
    #calculate least squares difference between solution and compare and return cost
    lsq = soln.position_array[:,0]-x_compare

    cost = np.sum(lsq)**2

    return cost

#Calibrate spring stiffness using Scipy Optimize

opt_soln = spo.minimize(
                        1dof_minize,
                        x0=10, 
                        args = (x_compare),
                        method="Nelder-Mead"
                        )

print(f"Optimal stiffness: {opt_soln.x}")

```

## Authors

[Turner Jennings](https://turnerjennings.github.io/)

## Version History

* 0.2
    * Various bug fixes
    * Nonlinear spring models
    * Linear/torsional spring coupling
* 0.1
    * Initial Release

## License

This project is licensed under the MIT License - see the LICENSE file for details
