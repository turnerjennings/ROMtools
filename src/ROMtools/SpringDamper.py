import numpy as np
from math import cos, sin

from ROMtools.RigidBody import RigidBody
from .RigidBody import *
import time
from typing import Tuple, Optional,Literal


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
        dof_constraint: Tuple[bool, bool, bool] = (False, False, False),
    ) -> None:
        if parent is not None:
            self.parent = parent.ID
            self.xp = parent_pos[0]
            self.yp = parent_pos[1]
        else:
            self.parent = -1
            self.xp = parent_pos[0]
            self.yp = parent_pos[1]

        if child is not None:
            self.child = child.ID
            self.xc = child_pos[0]
            self.yc = child_pos[1]
        else:
            self.child = -1
            self.xc = child_pos[0]
            self.yc = child_pos[1]

        self.k = float(k)
        self.c = float(c)
        self.p = float(p)

        self.constraint = dof_constraint

        # calculate initial length
        if parent is not None:
            t0p = parent.t0
            x0p = (
                parent.x0
                + parent_pos[0] * cos(parent.t0)
                + parent_pos[0] * sin(parent.t0)
            )
            y0p = (
                parent.y0
                + parent_pos[1] * cos(parent.t0)
                + parent_pos[1] * sin(parent.t0)
            )
        else:
            x0p = 0.0
            y0p = 0.0
            t0p = 0.0

        if child is not None:
            x0c = child.x0 + child_pos[0] * cos(child.t0) + child_pos[0] * sin(child.t0)
            y0c = child.y0 + child_pos[1] * cos(child.t0) + child_pos[1] * sin(child.t0)
            t0c = child.t0
        else:
            x0c = 0.0
            y0c = 0.0
            t0c = 0.0

        self.lx0 = abs(x0p - x0c)
        self.ly0 = abs(y0p - y0c)

        self.l0 = np.linalg.norm([self.lx0, self.ly0])
        self.l0t = t0p - t0c

    def calculate_force(self):
        raise RuntimeError("No force calculation defined for generic spring element")

    def _RelativePos(
        self, p: np.ndarray, v: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        # define variables from dof for convenience

        if v is not None:
            if self.parent >= 0:
                parent_x = p[3 * self.parent]
                parent_y = p[3 * self.parent + 1]
                parent_t = p[3 * self.parent + 2]

                parent_dx = v[3 * self.parent]
                parent_dy = v[3 * self.parent + 1]
                parent_dt = v[3 * self.parent + 2]
            else:
                parent_x = self.xp
                parent_y = self.yp
                parent_t = 0.0

                parent_dx = 0.0
                parent_dy = 0.0
                parent_dt = 0.0

            if self.child >= 0:
                child_x = p[3 * self.child]
                child_y = p[3 * self.child + 1]
                child_t = p[3 * self.child + 2]

                child_dx = v[3 * self.child]
                child_dy = v[3 * self.child + 1]
                child_dt = v[3 * self.child + 2]
            else:
                child_x = self.xc
                child_y = self.yc
                child_t = 0.0

                child_dx = 0.0
                child_dy = 0.0
                child_dt = 0.0
            vpx = parent_dx + parent_dt * (
                -self.xp * sin(parent_t) + self.yp * cos(parent_t)
            )
            vpy = parent_dy + parent_dt * (
                -self.yp * sin(parent_t) + self.xp * cos(parent_t)
            )

            vcx = child_dx + child_dt * (
                -self.xc * sin(child_t) + self.yc * cos(child_t)
            )
            vcy = child_dy + child_dt * (
                -self.yc * sin(child_t) + self.xc * cos(child_t)
            )

        else:
            if self.parent >= 0:
                parent_x = p[3 * self.parent]
                parent_y = p[3 * self.parent + 1]
                parent_t = p[3 * self.parent + 2]

            else:
                parent_x = 0.0
                parent_y = 0.0
                parent_t = 0.0

            if self.child >= 0:
                child_x = p[3 * self.child]
                child_y = p[3 * self.child + 1]
                child_t = p[3 * self.child + 2]
            else:
                child_x = 0.0
                child_y = 0.0
                child_t = 0.0

        # calculate parent and child relative positions
        rpx = parent_x + self.xp * cos(parent_t) - self.yp * sin(parent_t)
        rpy = parent_y + self.yp * cos(parent_t) + self.xp * sin(parent_t)

        rcx = child_x + self.xc * cos(child_t) - self.yc * sin(child_t)
        rcy = child_y + self.yc * cos(child_t) + self.xc * sin(child_t)

        # calculate torque moment arms
        armpx = rpx - parent_x
        armpy = rpy - parent_y

        armcx = rcx - child_x
        armcy = rcy - child_y

        if v is not None:
            return rpx, rpy, rcx, rcy, armpx, armpy, armcx, armcy, vpx, vpy, vcx, vcy
        else:
            return rpx, rpy, rcx, rcy, armpx, armpy, armcx, armcy


class LinearSpringDamper(SpringDamper):

    def __init__(
        self,
        k: float,
        c: float,
        p: float = 0,
        parent: RigidBody | None = None,
        child: RigidBody | None = None,
        parent_pos: Tuple[float] = (0, 0),
        child_pos: Tuple[float] = (0, 0),
        dof_constraint: Tuple[bool] = (False, False, False),
        type: Literal["Linear", "Tensile", "Compressive"] = "Linear"
    ) -> None:
        
        super().__init__(k, c, p, parent, child, parent_pos, child_pos, dof_constraint)
        self.type = type

    def calculate_force(self, p: np.ndarray, v: np.ndarray) -> np.ndarray:
        forces = np.zeros(p.shape)

        rpx, rpy, rcx, rcy, armpx, armpy, armcx, armcy, vpx, vpy, vcx, vcy = (
            self._RelativePos(p, v)
        )

        # compute relative displacement from relative position and initial lengths
        r_relx = rpx - rcx
        r_rely = rpy - rcy

        disp_mag = np.linalg.norm([r_relx, r_rely])
        
        if disp_mag > 0.0:
            unit_vec = np.array([r_relx, r_rely]) / disp_mag
            stretch = disp_mag + self.p - self.l0
        else:
            stretch = 0.0
            unit_vec = np.array([0.0,0.0])

        match self.type:
            case "Linear":
                force_mag = self.k * stretch
                F_kx = force_mag * unit_vec[0]
                F_ky = force_mag * unit_vec[1]
            case "Tensile":
                if stretch >= 0.0:
                    force_mag = self.k * stretch
                else:
                    force_mag = 0.0
                
                F_kx = force_mag * unit_vec[0]
                F_ky = force_mag * unit_vec[1]
            case "Compressive":
                if stretch <= 0.0:
                    force_mag = self.k * stretch
                else:
                    force_mag = 0.0
                
                F_kx = force_mag * unit_vec[0]
                F_ky = force_mag * unit_vec[1]



        # print(rpx, rpy, rcx, rcy, vpx, vpy, vcx, vcy)
        # time.sleep(0.1)

        # calculate torque moment arms

        T_pk = armpx * F_ky - armpy * F_kx
        T_ck = armcx * F_ky - armcy * F_kx

        # calculate parent and child relative velocities

        # compute relative displacement from relative position and initial lengths
        v_relx = vpx - vcx
        v_rely = vpy - vcy

        vel_mag = np.linalg.norm([v_relx, v_rely])

        if vel_mag > 0:

            unit_vec = np.array([v_relx, v_rely]) / vel_mag

            damp_mag = self.c * vel_mag

            F_cx = damp_mag * unit_vec[0]
            F_cy = damp_mag * unit_vec[1]
        else:
            damp_mag = 0
            F_cx = 0
            F_cy = 0

        T_pC = armpx * F_cy - armpy * F_cx
        T_cC = armcx * F_cy - armcy * F_cx

        if self.parent >= 0:

            forces[3 * self.parent] = (-F_kx - F_cx) * (1 - self.constraint[0])
            forces[3 * self.parent + 1] = (-F_ky - F_cy) * (1 - self.constraint[1])
            forces[3 * self.parent + 2] = (-T_pk - T_pC) * (1 - self.constraint[2])

        if self.child >= 0:

            forces[3 * self.child] = (F_kx + F_cx) * (1 - self.constraint[0])
            forces[3 * self.child + 1] = (F_ky + F_cy) * (1 - self.constraint[1])
            forces[3 * self.child + 2] = (T_ck + T_cC) * (1 - self.constraint[2])

        return forces


class RotationalSpringDamper(SpringDamper):

    def calculate_force(self, p: np.ndarray, v: np.ndarray) -> np.ndarray:
        forces = np.zeros(p.shape)

        # define variables from dof for convenience
        if self.parent >= 0:
            parent_t = p[3 * self.parent + 2]

            parent_dt = v[3 * self.parent + 2]
        else:
            parent_t = 0.0

            parent_dt = 0.0

        if self.child >= 0:
            child_t = p[3 * self.child + 2]

            child_dt = v[3 * self.child + 2]
        else:
            child_t = 0.0

            child_dt = 0.0

        # print(f"parent_t={parent_t}, child_t = {child_t}")
        T_k = self.k * (parent_t - child_t + self.p - self.l0t)
        T_c = self.c * (parent_dt - child_dt)

        forces[3 * self.parent + 2] = -T_k - T_c
        forces[3 * self.child + 2] = T_k + T_c

        return forces
