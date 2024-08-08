import numpy as np
from math import pi, cos, sin
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from typing import Tuple, Optional


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
        """Generic rigid body

        Args:
            ID (int): Identification number for the body
            m (float): Mass of body
            R (float): Radius of gyration of body (for moment of inertia calculation and rendering)
            name (str, optional): Optional body name for tracking on animation plot. Defaults to "".
            cm (Tuple[float, float, float], optional): Position of body center of mass. Defaults to (0.0, 0.0, 0.0).
            I (Optional[float], optional): Optional moment of inertia, I=m*R**2 otherwise. Defaults to None.
            dof_constraint (Tuple[bool,bool,bool], optional): Optional constraint of motion along certain degrees of freedom. Defaults to (False, False, False).
        """

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

    def initial_conditions(
        self,
        p: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        v: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        a: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """Apply initial position, velocity, or displacement to a rigid body

        Args:
            p (Tuple[float, float, float], optional): Initial displacement (relative to given center of mass). Defaults to (0.0, 0.0, 0.0).
            v (Tuple[float, float, float], optional): Initial velocity. Defaults to (0.0, 0.0, 0.0).
            a (Tuple[float, float, float], optional): Initial acceleration. Defaults to (0.0, 0.0, 0.0).
        """

        self.p0 = (p[0] + self.x0, p[1] + self.y0, p[2] + self.t0)
        self.v0 = v
        self.a0 = a

    def _inertia(self):
        return self.m * self.R**2


class Disc(RigidBody):

    def _inertia(self):
        return 0.5 * self.m * self.R**2

    def render_body(self, ax: Axes, p: np.ndarray, color: str = "blue"):
        xcen = p[0]
        ycen = p[1]
        theta = p[2]

        theta_array = np.linspace(0, 2 * pi, 51)
        head_points_x = xcen + self.R * np.cos(theta + theta_array)
        head_points_y = ycen + self.R * np.sin(theta + theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[25]])
        angle_line_y = np.array([head_points_y[0], head_points_y[25]])
        if self.name is not None:
            ax.plot(head_points_x, head_points_y, color=color, label=self.name)
        else:
            ax.plot(head_points_x, head_points_y, color=color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)


class Semicircle(RigidBody):

    def _inertia(self):
        return 0.5 * self.m * self.R**2

    def render_body(self, ax: Axes, p: np.ndarray, color: str = "red"):
        xcm = 0.0
        ycm = 0.0
        theta = 0.0

        xcm = p[0]
        ycm = p[1]
        theta = p[2]

        xcen = xcm + 4 * self.R / (3 * pi) * sin(theta)
        ycen = ycm - 4 * self.R / (3 * pi) * cos(theta)

        theta_array = np.linspace(0, pi, 50)
        head_points_x = xcen + self.R * np.cos(theta + theta_array)
        head_points_y = ycen + self.R * np.sin(theta + theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[-1]])
        angle_line_y = np.array([head_points_y[0], head_points_y[-1]])
        if self.name is not None:
            ax.plot(head_points_x, head_points_y, color=color, label=self.name)
        else:
            ax.plot(head_points_x, head_points_y, color=color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)


class circle(RigidBody):

    def _inertia(self):
        return self.m * self.R**2

    def render_body(self, ax: Axes, p: np.ndarray, color: str = "green"):
        xcen = p[0]
        ycen = p[1]
        theta = p[2]

        theta_array = np.linspace(0, 2 * pi, 51)
        head_points_x = xcen + self.R * np.cos(theta + theta_array)
        head_points_y = ycen + self.R * np.sin(theta + theta_array)

        angle_line_x = np.array([head_points_x[0], head_points_x[25]])
        angle_line_y = np.array([head_points_y[0], head_points_y[25]])
        if self.name is not None:
            ax.plot(head_points_x, head_points_y, color=color, label=self.name)
        else:
            ax.plot(head_points_x, head_points_y, color=color)

        ax.plot(angle_line_x, angle_line_y, color="black", alpha=0.5)
