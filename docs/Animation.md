# Animator

Handler for creating animated output plots illustrating model dynamics.

## Attributes


| Attribute |   Type   | Description|
|---------|-------------|-------------|
| time | np.ndarray | Solver timesteps |
| n_timesteps | int | Number of timesteps in the analysis |
| n_frames | int | Number of frames to render |
| pos | np.ndarray | Position array from solution |
| fps | int | Frames per second for saved animation |
| concise | bool | Option to animate only dynamics system motion (True) or animate system motion plus position vs time plots (False) |
| body_colors | list [string] | List of colors to render bodies from solver |
| body_names | list [string] | List of body nicknames for legend rendering |
| body_ID | list [int] | IDs for each body in the analysis |
| springs | list [SpringDamper] | List of springs included in the analysis |
| xlim, ylim, tlim | tuple [float, float] | Min and Max limits for animator plots in x, y, and theta |
| metadata | dict (optional) | Optional metadata for rendered video |
| path | string | save path for output render |

# Methods

## ____init____

```python
class Animator:

    def __init__(
        self,
        sol: Solver,
        fps: int = 15,
        metadata: Optional[dict] = None,
        xlim: Tuple[float, float] = None,
        ylim: Tuple[float, float] = None,
        tlim: Tuple[float, float] = None,
    ) -> None:

#create animator from existing solution
anim = Animator(solver)
```

__inputs__
| Attribute |   Type   | Description|
|---------|-------------|-------------|
| sol | Solver | Solver result to animate |
| fps | int (optional) | Frames per second to render video at. Defaults to 15 |
| metadata | dict (optional) | Optional metadata to include in render |
| xlim, ylim, tlim (optional) | tuple [float, float] | Option to manually specify x, y, theta limits for the animation window.  Limits will be automatically calculated if not specified |

__Returns__
Animator object

## __Animate__

Render results from Animator and saves to file specified in solver/run configuration

```python

#create animator from existing solution
anim = Animator(solver)

#render result
anim.animate()
```
