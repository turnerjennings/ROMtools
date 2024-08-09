import numpy as np
from math import cos, sin

from ROMtools.RigidBody import RigidBody
from .RigidBody import *
import time
from typing import Tuple, Optional, Literal


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
        track_force:bool = False,
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
                - parent_pos[0] * sin(parent.t0)
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
            x0c = child.x0 + child_pos[0] * cos(child.t0) - child_pos[0] * sin(child.t0)
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

        #force tracking
        if track_force == True:
            self.forcehist = []
        else:
            self.forcehist= None


    def calculate_force(self):
        raise RuntimeError("No force calculation defined for generic spring element")

    def _RelativePos(
        self, p: np.ndarray, v: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float, float]:
        # define variables from dof for convenience


        #load position info
        parent_p = np.array([self.xp, self.yp]).reshape((2,1))
        child_p = np.array([self.xp, self.yp]).reshape((2,1))



        if self.parent >= 0:
            parent_x = p[3 * self.parent:3 * self.parent + 2].reshape((2,1))
            theta = p[3 * self.parent + 2]
            parent_R = np.array([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]])


        else:
            parent_x = np.zeros(2)
            parent_R = np.eye(2)


        if self.child >= 0:

            child_x = p[3 * self.child:3 * self.child + 2].reshape((2,1))
            theta = p[3 * self.child + 2]
            parent_R = np.array([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]])

        else:
            child_x = np.zeros(2).reshape((2,1))
            child_R = np.eye(2)

        #load velocity info
        parent_dx = np.array([0.0,0.0]).reshape((2,1))
        parent_w = np.zeros((2,2))

        child_dx = np.array([0.0,0.0]).reshape((2,1))
        child_w = np.zeros((2,2))
        
        if v is not None:
            if self.parent >= 0:

                parent_dx = v[3 * self.parent:3 * self.parent + 2].reshape((2,1))
                wz = v[3 * self.parent + 2]
                parent_w = np.array([[0,-wz],[wz,0]])

            if self.child >= 0:

                child_dx = v[3 * self.child:3 * self.child + 2].reshape((2,1))
                wz = v[3 * self.child + 2]
                child_w = np.array([[0,-wz],[wz,0]])

        #calculate position
        
        #moment arms
        arm_p =  parent_R@parent_p
        arm_c = child_R@child_p
        #positions        
        r_p = arm_p + parent_x   
        r_c = arm_c + child_x  

        #calculate velocities
        v_p = parent_w@parent_R@parent_p+parent_dx
        v_c = child_w@child_R@child_p+child_dx



        if v is not None:
            return r_p, r_c, arm_p, arm_c, v_p, v_c
        else:
            return r_p, r_c, arm_p, arm_c,


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
        track_force:bool = False,
        type: Literal["Linear", "Tensile", "Compressive", "Tabular"] = "Linear",
        curve: np.ndarray = None,
    ) -> None:

        super().__init__(k, c, p, parent, child, parent_pos, child_pos, dof_constraint, track_force)
        self.type = type

        if self.type == "Tabular":
            if curve is not None:
                self.kcurve = curve
            else:
                raise ValueError(
                    "Input argument Curve required for tabular spring type"
                )
        else:
            self.kcurve = None

    def calculate_force(self, p: np.ndarray, v: np.ndarray) -> np.ndarray:
        forces = np.zeros(p.shape)

        r_p, r_c, arm_p, arm_c, v_p, v_c = self._RelativePos(p,v)

        # compute relative displacement from relative position and initial lengths
        r_rel = r_p - r_c

        disp_mag = np.linalg.norm(r_rel)

        if disp_mag > 0.0:
            unit_vec = r_rel / disp_mag
            stretch = disp_mag + self.p - self.l0
        else:
            stretch = 0.0
            unit_vec = np.array([0.0, 0.0])


        
        # calculate force depending on type
        if self.type == "Linear":
            force_mag = self.k * stretch
            F_k = force_mag * unit_vec
        elif self.type == "Tensile":
            if stretch >= 0.0:
                force_mag = self.k * stretch
            else:
                force_mag = 0.0

            F_k = force_mag * unit_vec
        elif self.type == "Compressive":
            if stretch <= 0.0:
                force_mag = self.k * stretch
            else:
                force_mag = 0.0

            F_k = force_mag * unit_vec

        elif self.type == "Tabular":
            force_mag = np.interp(abs(stretch), self.kcurve[:, 0], self.kcurve[:, 1])
            if stretch < 0:
                force_mag = -1 * force_mag

            F_k = force_mag * unit_vec

        # print(rpx, rpy, rcx, rcy, vpx, vpy, vcx, vcy)
        # time.sleep(0.1)

        # calculate torque moment arms

        T_pk = np.cross(arm_p.reshape(2,),F_k.reshape(2,))
        T_ck = np.cross(arm_c.reshape(2,),F_k.reshape(2,))

        # calculate parent and child relative velocities

        # compute relative displacement from relative position and initial lengths
        v_rel = v_p - v_c

        vel_mag = np.linalg.norm(v_rel)

        if vel_mag > 0:

            unit_vec = v_rel / vel_mag

            damp_mag = self.c * vel_mag
        else:
            damp_mag = 0
        
        
        F_c = damp_mag * unit_vec


        T_pC = np.cross(arm_p.reshape(2,),F_c.reshape(2,))
        T_cC = np.cross(arm_c.reshape(2,),F_c.reshape(2,))

        
        #calculate force balance and update force array
        F_totx = F_k[0] + F_c[0]
        F_toty = F_k[1] + F_c[1]
        T_totp = T_pk + T_pC
        T_totc = T_ck + T_cC
        
        
        if self.parent >= 0:

            forces[3 * self.parent] = (-F_totx) * (1 - self.constraint[0])
            forces[3 * self.parent + 1] = (-F_toty) * (1 - self.constraint[1])
            forces[3 * self.parent + 2] = (T_totp) * (1 - self.constraint[2])

        if self.child >= 0:

            forces[3 * self.child] = (F_totx) * (1 - self.constraint[0])
            forces[3 * self.child + 1] = (F_toty) * (1 - self.constraint[1])
            forces[3 * self.child + 2] = (T_totc) * (1 - self.constraint[2])

        return forces


class RotationalSpringDamper(SpringDamper):

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
        track_force:bool = False,
        type: Literal["Linear","Linked"] = "Linear",
        link: Optional[LinearSpringDamper] = None,
        linkcoef: Tuple[float] = (1,1)
    ) -> None:
        super().__init__(k, c, p, parent, child, parent_pos, child_pos, dof_constraint,track_force)

        self.type = type
        if self.type == "Linked":
            if link is not None:
                self.link = link
                self.linkingcoef = linkcoef
            else:
                raise ValueError("Linked linear spring must be specified for Linked type")


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
            
        if self.type == "Linear":
            T_k = self.k * (parent_t - child_t + self.p - self.l0t)
            T_c = self.c * (parent_dt - child_dt)
        elif self.type == "Linked":
            linspringforce = self.link.calculate_force(p,v)
            forcemag = np.linalg.norm([linspringforce[3 * self.parent],linspringforce[3 * self.parent + 1]])

            T_k =  forcemag*self.linkingcoef[0] * (parent_t - child_t + self.p - self.l0t)
            T_c =  forcemag*self.linkingcoef[0] * (parent_dt - child_dt)

        T_tot = T_k + T_c

        forces[3 * self.parent + 2] = -T_tot
        forces[3 * self.child + 2] = T_tot

        return forces
