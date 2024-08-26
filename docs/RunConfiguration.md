# Run Configuration

Dataclass to store settings governing how a solver is executed.

## Attributes

| Attribute | Type | Description|
|---------|-------------|-------------|
| output_path | string | Filepath to save output files to |
| output_name | string | Run name forming prefix for all output files | 
| output_as_csv | bool | Save output files as .csv (True) or as .npy (False), defaults to True | 
| output_position | bool | Option to save the calculated position data. Defaults to True | 
| output_velocity | bool | Option to save the calculated velocity data. Defaults to True |
| output_acceleration | bool | Option to save the calculated force data. Defaults to True |
| Solver_type | "RK4", "FwdEuler", or "CentralDifference" | Selected time integration method, see Solver for details |
| termination_time | float | End time for simulation |
| n_timesteps | int | number of timesteps to calculate between (0,termination_time), defaults to 1000 |
| render_name | string | Name prefix for animator file, defaults to output_name |
| animate_concise | bool | Option to animate only dynamics system motion (True) or animate system motion plus position vs time plots (False).  Defaults to True |
| bodies_to_render | List [int] | List of bodies to include when animating results. defaults to all bodies |
| body_colors | List [string] | List of colors to render bodies when animating |
| n_frames | int | number of frames to generate out of full simulation length. Defaults to 100 |

---

## Methods

### __update_from_dict__

update run properties from a dictionary
```python

config_dict = {
    "output_path": "C:/path/to/output/file/",
    "output_name": "example_run",
    "solver_type": "RK4",
    "termination_time": 0.05,
    "n_timesteps": 10000,
    "animate_concise": True
}

#create configuration and update from dictionary
run_config = RunConfiguration()
run_config.update_from_dict(config_dict=config_dict)

```
__inputs__

| Attribute | Type | Description|
|---------|-------------|-------------|
| config_dict | dict | dictionary containing name value pairs according to the attributes table above |

__returns__

RunConfiguration object

---

### __save_to_file__

Save run configuration to .json file for use in later simulations

```python

config_dict = {
    "output_path": "C:/path/to/output/file/",
    "output_name": "example_run",
    "solver_type": "RK4",
    "termination_time": 0.05,
    "n_timesteps": 10000,
    "animate_concise": True
}

#create configuration and update from dictionary
run_config = RunConfiguration()
run_config.update_from_dict(config_dict=config_dict)

#save to file
run_config.save_to_file("save/path/settings.json")
```

| Attribute | Type | Description|
|---------|-------------|-------------|
| filename | string | Filename to save run config |

__Returns__

None

---

### __load_from_file__

Load run configuration from existing .json file

```python

#create configuration and update from file
run_config = RunConfiguration()

run_config.load_from_file("save/path/settings.json")
```

| Attribute | Type | Description|
|---------|-------------|-------------|
| filename | string | Filename to load run config |

__Returns__

None

