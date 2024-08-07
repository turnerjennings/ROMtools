import numpy as np
from math import pi,cos,sin
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

class RigidBody():

    def __init__(self, ID:int, m:float, R:float, name:str = "", cm:tuple[float, float, float] = (0.0, 0.0, 0.0), I:float = None, dof_constraint:tuple[bool,bool,bool] = (False, False, False)) -> None:
        self.ID = ID
        self.name = name
        self.m = m
        self.R = R
        self.x0 = cm[0]
        self.y0 = cm[1]
        self.t0 = cm[2]
        self.constraint = dof_constraint

        if I is not None:
            self.I = I
        else:
            self.I = self._inertia()
        
        self.p0 = (cm[0], cm[1], cm[2])
        self.v0 = (0.0, 0.0, 0.0)
        self.a0 = (0.0, 0.0, 0.0)
        
        

    def __str__(self) -> str:
        return f"Rigid body {self.ID}:\n\tMass: {self.m}\n\tInertia: {self.I}\n\tCOM: ({self.x0}, {self.y0})"
    
    def initial_conditions(self, p:tuple[float, float, float]=(0.0, 0.0, 0.0), v:tuple[float, float, float]=(0.0, 0.0, 0.0), a:tuple[float, float, float]=(0.0, 0.0, 0.0)):
        self.p0 = (p[0] + self.x0, p[1] + self.y0, p[2] + self.t0)
        self.v0 = v
        self.a0 = a
    
    def _inertia(self):
        return self.m * self.R**2

        

class Disc(RigidBody):

    def _inertia(self):
        return 0.5*self.m*self.R**2
    
    def render_body(self,ax:Axes,p:np.ndarray, color:str="blue"):
        xcen=p[0]
        ycen=p[1]
        theta=p[2]

        theta_array = np.linspace(0,2*pi,51)
        head_points_x = xcen + self.R * np.cos(theta+theta_array)
        head_points_y = ycen + self.R * np.sin(theta+theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[25]])
        angle_line_y = np.array([head_points_y[0], head_points_y[25]])
        if self.name is not None:
            ax.plot(head_points_x,head_points_y, color = color, label = self.name)
        else:
            ax.plot(head_points_x,head_points_y, color = color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)


class Semicircle(RigidBody):

    def _inertia(self):
        return 0.5*self.m*self.R**2
    
    def render_body(self,ax:Axes,p:np.ndarray, color:str = "red"):
        xcen=p[0]
        ycen=p[1]
        theta=p[2]

        theta_array = np.linspace(0,pi,50)
        head_points_x = xcen + self.R * np.cos(theta+theta_array)
        head_points_y = ycen + self.R * np.sin(theta+theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[-1]])
        angle_line_y = np.array([head_points_y[0], head_points_y[-1]])
        if self.name is not None:
            ax.plot(head_points_x,head_points_y, color = color, label = self.name)
        else:
            ax.plot(head_points_x,head_points_y, color = color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)
    
    
class Semicircle(RigidBody):

    def _inertia(self):
        return self.m*self.R**2
    
    def render_body(self,ax:Axes,p:np.ndarray, color:str = 'green'):
        xcen=p[0]
        ycen=p[1]
        theta=p[2]

        theta_array = np.linspace(0,2*pi, 51)
        head_points_x = xcen + self.R * np.cos(theta+theta_array)
        head_points_y = ycen + self.R * np.sin(theta+theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[25]])
        angle_line_y = np.array([head_points_y[0], head_points_y[25]])
        if self.name is not None:
            ax.plot(head_points_x,head_points_y,color = color, label = self.name)
        else:
            ax.plot(head_points_x,head_points_y, color = color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)

        
