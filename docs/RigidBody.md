# Rigid Bodies


A class to define rigid objects of interest to perform rigid body dynamics simulation on.  All calculations are performed treating the x-, y-, and theta- movement of the center of mass as the degrees of freedom for the body.

---

## Attributes


| Attribute | Type | Description|
|---------|-------------|-------------|
| ID | Int | Identifier number for rigid body |
| name | String | Nickname for the rigid body (used for labelling in Animator) |
| m | float | Rigid body mass |
| R | float | Rigid body radius (or radius of gyration for generalized objects) |
| I | float | Rigid body mass moment of inertia |
| x0 | float | Initial x-coordinate of the center of mass |
| y0 | float | Initial y-coordinate of the center of mass |
| t0 | float | Initial theta-orientation of the rigid body |
| constraint | tuple[bool, bool, bool] | Degree of freedom constraints applied to the body (true = motion constrained in that degree of freedom) |
| p0 | tuple[float, float, float] | Initial position of the rigid body |
| v0 | tuple[float, float, float] | Initial velocity of the rigid body |
| a0 | tuple[float, float, float] | Initial acceleration of the rigid body |

---

## Subclasses

All subclasses share the same methods detailed below, but configure a pre-specified rigid body for simplicity.  Selection of pre-specified rigid body geometry affects moment of inertia calculation and animation rendering:

- Disc
- Semicircle
- Circle

---

## Methods


### __init__

Initialize a new rigid body.  __init__ method is identical for all subclasses.

``` python
class RigidBody:

    def __init__(
        self,
        ID: int,
        m: float,
        R: float,
        name: str = "",
        cm: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        I: Optional[float] = None,
        dof_constraint: Tuple[bool, bool, bool] = (False, False, False),
    ) -> None:

#Define a solid disc rigid body with ID 0 located at the origin with no motion in the x-direction
disc = Disc(ID=0, m=1.12, R=1.56, name = "body 1", cm = (0.0,0.0,0.0), dof_constraint = (True, False, False))
```
__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| ID | Int | ID for keyword object |
| m | Float | Rigid body mass |
| R | Float | Rigid body radius of gyration |
| Name | String (optional) | Nickname for body, defaults to none |
| cm | Tuple[float, float, float] (optional) | Center of mass of body, defaults to (0,0,0) |
| I | Float (optional) | Rigid body moment of Inertia, if not specified, calculation depends on body type |
| dof_constraint | Tuple[bool, bool, bool] (optional) | Degree-of-freedom constraints for body |


__returns__

Array Keyword object

__notes__

If a moment of inertia is not specified, the moment of inertia is calculated automatically using the appropriate formulation for the body type:

- RigidBody (treated as a thin circle): $I=m*R^2$
- Disc: $I=\frac{m*R^2}{2}$
- Semicircle: $I=\frac{m*R^2}{2}$
- circle: $I=m*R^2$

---

### __initial_conditions__

Apply initial conditions to a rigid body

``` python
#Define a solid disc rigid body with ID 0 located at the origin with no motion in the x-direction
disc = Disc(ID=0, m=1.12, R=1.56, name = "body 1", cm = (0.0,0.0,0.0), dof_constraint = (True, False, False))

#apply an initial displacement in the y-direction and an initial velocity in the theta direction
disc.initial_conditions(p=(0.0, 1.0, 0.0), v=(0.0, 0.0, 0.5))
```

__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| p | Tuple[float, float, float] (optional) | Initial displacement, defaults to (0,0,0) |
| v | Tuple[float, float, float] (optional) | Initial velocity, defaults to (0,0,0) |
| a | Tuple[float, float, float] (optional) | Initial acceleration, defaults to (0,0,0) |


__returns__

None. Updates object attributes for initial conditions.

---

### __render_body__

Render the body in a given configuration on a plot.  Used in animation but can be called separately to render specific configurations

``` python
#Define a solid disc rigid body with ID 0 located at the origin with no motion in the x-direction
disc = Disc(ID=0, m=1.12, R=1.56, name = "body 1", cm = (0.0,0.0,0.0), dof_constraint = (True, False, False))

#apply an initial displacement in the y-direction and an initial velocity in the theta direction
disc.initial_conditions(p=(0.0, 1.0, 0.0), v=(0.0, 0.0, 0.5))

#create a plot
fig, ax = plt.subplots()

ax = plt.subplot(1,1,1)

#plot the disc at (0,1) with an angle of 0.5 radians
disc.render_body(ax=ax, p=(0,1,0.5), color = "red")
```

__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| ax | matplotlib.pyplot.Axes | Axes object to render the body on |
| p | Tuple[float, float, float] | Position and orientation to render the object at|
| color | String (optional) | Color to render body, default depends on subclass |


__returns__

None.
