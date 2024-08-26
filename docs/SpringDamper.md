# Spring/Dampers
A class to define non-rigid constraints between bodies using linear or rotational spring/damper series.

---

## Core Attributes

The attributes below are universal to all springtypes.

| Attribute |   Type   | Description|
|---------|-------------|-------------|
| k | float | Linear spring constant |
| c | float | Linear damping constant |
| p | float | initial spring compression |
| constraint | tuple [bool, bool, bool] | ($x$, $y$, $\theta$) spring/damper constraints, if True then spring/damper does not act along the specified axis |
| parent | int | Body 1 connected to spring/damper.  Relative angles and positions measured relative to this body |
| child | int | Body 2 connected to spring/damper |
| xp, yp | float | Coordinates of parent body connection relative to parent body center of mass.  If parent body is None, then absolute coordinates of connection to rigid boundary |
| xc, yc | float | Coordinates of child body connection relative to child body center of mass.  If child body is None, then absolute coordinates of connection to rigid boundary |
| l0x, l0y, l0t | float | initial length (or angle) between the parent and child body.  Spring displacements are measured relative to this property. |
| forcehist | np.ndarray | Array of shape (n_timestep, 4), with columns tracking the $x-$ and $y-$ force applied to the bodies, as well as the $\theta_p-$ and $\theta_c-$ torque applied to the parent and child bodies. |

```python
class SpringDamper:

    def __init__(
        self,
        k: float,
        c: float,
        p: float = 0.0,
        parent: Optional[RigidBody] = None,
        child: Optional[RigidBody] = None,
        parent_pos: Tuple[float, float] = (0.0, 0.0),
        child_pos: Tuple[float, float] = (0.0, 0.0),
        dof_constraint: Tuple[bool, bool, bool] = (False, False, False)
    ) -> None:
```

---



# Linear Spring/Dampers

Linear spring/dampers are a generic element which can apply x- and y- forces to their parent and child bodies.  Torque can be applied to the parent and child bodies directly by defining the connection coordinates to be eccentric on the parent or child body.  Additional torque transfer can be defined through defining a spring force and damping force coupling coefficients, which enable the transfer of the normal spring/damper force to a tangential load similar to frictional effects.


## Attributes

The attributes below are specific to the linear spring/damper class:

| Attribute | Type | Description|
|---------|-------------|-------------|
| type | "Linear", "Tensile", "Compressive", "Tabular" | Spring force calculation method, see notes below |
| mu | tuple [float, float] | Rotational wet and dry coupling coefficents, see notes below |
| kcurve | np.ndarray | Force vs. displacement curve for tabular formulation, see notes below |

## Methods

## ____init____
Initialize a new linear spring/damper.

``` python
class LinearSpringDamper(SpringDamper):

    def __init__(
        self,
        k: float,
        c: float,
        p: float = 0,
        parent: Optional[RigidBody] = None,
        child: Optional[RigidBody] = None,
        parent_pos: Tuple[float] = (0, 0),
        child_pos: Tuple[float] = (0, 0),
        dof_constraint: Tuple[bool] = (False, False, False),
        track_force: bool = False,
        type: Literal["Linear", "Tensile", "Compressive", "Tabular"] = "Linear",
        curve: np.ndarray = None,
        mu:Tuple[float] = (0.0,0.0)
    ) -> None:

#Define a spring connecting two rigid bodies, 
#initially compressed and connected eccentrically to the child body, and track the force history
spring12 = LinearSpringDamper(k=1000, c=10, p=-0.001, 
                              parent=body1, child=body2, child_pos=(0.1, 0))

#Define a compression-only spring with rotational coupling connected to the boundary
spring12 = LinearSpringDamper(k=1000, c=10, p=-0.001, 
                              parent=body1, type = "Compressive", 
                              mu=(0.05, 0.001))
```
__Inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| k | float | Linear spring constant |
| c | float | Linear damping constant |
| p | float | Initial spring displacement (positive = tension) |
| parent | RigidBody | Connected body 1 (optional) |
| child | RigidBody | Connected body 2 (optional) |
| parent_pos | tuple [float, float] (optional) | x- and y- position of spring connection to parent body, defaults to None |
| child_pos | tuple [float, float] (optional) | x- and y- position of spring connection to child body, defaults to None |
| dof_constraint | tuple [bool, bool, bool] (optional) | degrees of freedom which the spring acts along, defaults to all DOF active |
| type | "Linear", "Tensile", "Compressive", "Tabular" (optional) | Spring formulation, see notes below, defaults to Linear |
| curve | np.ndarray (optional) | Force vs. displacement curve for tabular formulation, defaults to None |
| mu | tuple [float, float] (optional) | Wet and dry rotational coupling, defaults to no coupling (0,0) |

__Returns__

LinearSpringDamper object

__Notes__
---

__*Spring displacement calculation:*__

Each spring/damper connects to a parent body (denoted $x_p$)to a child body (denoted $x_c$).  A generic linear spring/damper connecting two bodies at an arbitrary point on each body $(\delta x_p, \delta y_p)$ and $(\delta x_c, \delta y_c)$, the relative position and velocity of the connection points is given by:

>$\vec{r}_p = \langle x_p + \delta x_p \cos{\theta_p} - \delta y_p \sin{\theta_p}, y_p + \delta y_p \cos{\theta_p} + \delta x_p \sin{\theta_p}\rangle$

>$\vec{v}_p = \langle \dot{x}_p + \dot{\theta}_p (- \delta x_p\sin{\theta_p} - \delta y_p \cos{\theta_p}), \dot{y}_p + \dot{\theta}_p (-\delta y_p \sin{\theta_p} + \delta x_p \cos{\theta_p})\rangle$

Subsequently, the relative displacement and velocity of the parent and child attachment points are given by:

>$\vec{r}_{rel} = \langle r_{px}-r_{cx}-l_{0x}, r_{py}-r_{cy}-l_{0y} \rangle = \langle \Delta r_x, \Delta r_y \rangle$

>$\vec{v}_{rel} = \langle v_{px}-v_{cx}, v_{py}-v_{cy} \rangle = \langle \Delta v_x, \Delta v_y \rangle$

Spring and damper forces for each spring are calculated from the relative lengths and velocities calculated above.

__*Spring types:*__

Four formulations are available for the linear spring/damper.  The "Linear" option relates force to displacement using Hooke's law:

>$\vec{F}_k=k(||\vec{r}_{rel}||-l_0)$

>$\vec{F}_c=c||\vec{v}_{rel}||$

The "Compressive" option relates force to displacement using Hooke's law only when the spring stretch is less than zero.  The "Tensile" option similarly only applies a force when the spring is in Tension.

The "Tabular" option allows the definition of an arbitrary force vs. displacement curve.  Force will be interpolated along the curve during each force calculation.  __this formulation is a work in progress__

__*Rotational coupling:*__

Torque may be applied by the spring due to an eccentric connection to the body creating a moment.  Additional coupling can be prescribed by specifying values $\mu_k$ and $\mu_c$ which transfer a portion of the spring and damper forces to a torque on the body.  The coefficients $\mu_k$ and $\mu_c$ are analogous to specifying a moment arm which the spring and damper forces will act on on the body:

>$T_{spring} = \vec{r} \times \vec{F}_{spring} + \mu_k ||\vec{F}_{spring}||(\theta_p - \theta_c) - \mu_c ||\vec{F}_{spring}||(\omega_p - \omega_c)$

---

# Rotational Spring/Damper

Rotational spring/dampers are a generic element which can apply Torque to bodies based on their relative angle and angular velocity.

## Methods

## ____init____
Initialize a new rotational spring/damper.

``` python
class RotationalSpringDamper(SpringDamper):

    def __init__(
        self,
        k: float,
        c: float,
        p: float = 0,
        parent: Optional[RigidBody] = None,
        child: Optional[RigidBody] = None,
        dof_constraint: Tuple[bool] = (False, False, False),
    ) -> None:

#Define a rotational spring connecting two rigid bodies
spring12 = RotationalSpringDamper(k=1000, c=10, p=-0.001, 
                              parent=body1, child=body2)

```
__Inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| k | float | Linear spring constant |
| c | float | Linear damping constant |
| p | float | Initial spring displacement (positive = tension) |
| parent | RigidBody | Connected body 1 (optional) |
| child | RigidBody | Connected body 2 (optional) |
| dof_constraint | tuple [bool, bool, bool] (optional) | degrees of freedom which the spring acts along, defaults to all DOF active |

__Returns__

Rotational spring/damper object