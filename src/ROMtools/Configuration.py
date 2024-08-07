from dataclasses import dataclass
import json

@dataclass
class RunConfiguration:
    #output settings
    output_path: str = "T:/codes/ROM_rbd/scripts/"
    output_name: str = "run"

    output_as_csv: bool = True

    output_position: bool = True
    output_velocity: bool = True
    output_acceleration: bool = True
    output_force: bool = True
    
    #general settings
    solver_type: str = "RK4"
    output_frequency: int = 10


    #timing information
    termination_time: float = 1.0
    n_timesteps: int = 1000

    #time integration settings
    alpha: float = 0.1

    #Animation settings
    render_name: str = output_name
    animate_concise:bool = False
    bodies_to_render: list[int] = None
    body_colors: list[str] = None
    n_frames: int = 100
    
    
    
    
    def update_from_dict(self, config_dict: dict[str, any]) -> None:
        for key, value in config_dict.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise KeyError(f"Invalid configuration key: {key}")
            
    def save_to_file(self, filename: str) -> None:
        with open(filename, 'w') as file:
            json.dump(self.__dict__, file, indent=4)
    
    @classmethod 
    def load_from_file(cls, filename: str) -> "RunConfiguration":
        with open(filename, 'r') as file:
            config_dict = json.load(file)
        config = cls()
        config.update_from_dict(config_dict)
        return config