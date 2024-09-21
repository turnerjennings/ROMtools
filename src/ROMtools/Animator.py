import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from .Solver import *
from matplotlib.gridspec import GridSpec
from matplotlib.axes import Axes
from typing import Tuple, Optional


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
        """Class which handles animation of the results from a given solver.

        Args:
            sol (Solver): Solver to animate
            fps (int, optional): Frames per second for output video. Defaults to 15.
            metadata (Optional[dict], optional): Metadata for video file. Defaults to None.
            xlim (Tuple[float, float], optional): tuple of x-limits for animation window. Defaults to None.
            ylim (Tuple[float, float], optional): tuple of y-limits for animation window. Defaults to None.
            tlim (Tuple[float, float], optional): tuple of t-limits for animation windwo. Defaults to None.
        """
        # check if solution exists
        if sol.solved == False:
            raise ValueError("Cannot animate solver with no solution")

        # basic data
        self.time = sol.timesteps
        self.n_timesteps = sol.n_timesteps
        self.n_frames = sol.config.n_frames
        self.pos = sol.position_array
        self.fps = fps
        self.concise = sol.config.animate_concise

        # body data
        self.body_colors = sol.config.body_colors
        self.body_names = [body.name for body in sol.bodies]

        if sol.config.bodies_to_render is None:
            self.bodies = sol.bodies
            self.body_ID = [body.ID for body in self.bodies]
        else:
            self.bodies = sol.bodies[sol.config.bodies_to_render]
            self.body_ID = [
                body.ID for body in self.bodies[sol.config.bodies_to_render]
            ]

        self.springs = sol.springs

        # limits
        sf = 1.5
        radii = np.array([body.R for body in self.bodies])
        if xlim is not None:
            self.xlim = xlim
        else:
            self.xlim = (
                np.min(sol.position_array[:, ::3]) - sf * np.max(radii),
                np.max(sol.position_array[:, ::3]) + sf * np.max(radii),
            )

        if ylim is not None:
            self.ylim = ylim
        else:
            self.ylim = (
                np.min(sol.position_array[:, 1::3]) - sf * np.max(radii),
                np.max(sol.position_array[:, 1::3]) + sf * np.max(radii),
            )

        if tlim is not None:
            self.tlim = tlim
        else:
            self.tlim = (
                np.min(sol.position_array[:, 2::3]),
                np.max(sol.position_array[:, 2::3]),
            )

        self.timelim = (np.min(self.time), np.max(self.time))

        self.xvlim = (
            np.min(sol.position_array[:, ::3]),
            np.max(sol.position_array[:, ::3]),
        )
        self.yvlim = (
            np.min(sol.position_array[:, 1::3]),
            np.max(sol.position_array[:, 1::3]),
        )
        self.tvlim = (
            np.min(sol.position_array[:, 2::3]),
            np.max(sol.position_array[:, 2::3]),
        )

        # movie metadata
        if metadata is not None:
            self.metadata = metadata
        else:
            self.metadata = {"title": sol.config.render_name, "artist": "ROM_RBD"}

        self.path = sol.config.output_path + sol.config.output_name

    def animate(self):
        """Render animation from solver output"""
        if self.concise == True:
            self._animate_concise()
        else:
            self._animate_grid()

    def _animate_concise(self):
        """Render animation with only the movie window"""
        writer = PillowWriter(fps=self.fps, metadata=self.metadata)

        fig, ax = plt.subplots(1, 1, figsize=(7, 7))
        fig.tight_layout()

        ax = plt.subplot(1, 1, 1)

        frames = np.arange(
            0, self.n_timesteps, self.n_timesteps / self.n_frames, dtype=int
        )

        with writer.saving(fig, self.path + "_anim.gif", 100):
            for idx, i in enumerate(frames):
                ax.cla()

                if idx % 5 == 0:
                    print(f"Rendering frame {idx}...")

                for j, body in enumerate(self.bodies):
                    # print(f"j={j}, body = {body}, body ID = {body.ID}")
                    # extract plot label
                    if self.body_names[j] is not None:
                        label = self.body_names[j]
                    else:
                        label = None

                    body_pos = self.pos[
                        frames[: idx + 1], 3 * body.ID : 3 * body.ID + 3
                    ]
                    # print(body_pos)

                    if self.body_colors is not None:
                        color = self.body_colors[j]
                        ax.plot(
                            body_pos[:, 0], body_pos[:, 1], color=color, label=label
                        )
                        body.render_body(ax, body_pos[-1, :], color)
                    else:
                        ax.plot(body_pos[:, 0], body_pos[:, 1], label=label)
                        body.render_body(ax, body_pos[-1, :])

                for j, spring in enumerate(self.springs):
                    if type(spring) == LinearSpringDamper:
                        xp, yp, xc, yc, _, _, _, _ = spring._RelativePos(self.pos[i, :])
                        ax.plot(np.array([xp, xc]), np.array([yp, yc]), color="red")

                ax.set_xlim(self.xlim[0], self.xlim[1])
                ax.set_ylim(self.ylim[0], self.ylim[1])
                ax.legend()
                ax.set_title(f"Time = {self.time[i]:.4f}")

                writer.grab_frame()

    def _animate_grid(self):
        """Render animation with movie window and x/y/t vs time plots"""
        writer = PillowWriter(fps=self.fps, metadata=self.metadata)

        fig = plt.figure(figsize=(20, 10))
        gs = GridSpec(3, 6, figure=fig)
        ax_anim = fig.add_subplot(gs[:, 0:3])
        ax_xpos = fig.add_subplot(gs[0, 3:])
        ax_ypos = fig.add_subplot(gs[1, 3:])
        ax_tpos = fig.add_subplot(gs[2, 3:])

        frames = np.arange(
            0, self.n_timesteps, self.n_timesteps / self.n_frames, dtype=int
        )

        with writer.saving(fig, self.path + "_anim.gif", 100):
            for idx, i in enumerate(frames):
                ax_anim.cla()
                ax_xpos.cla()
                ax_ypos.cla()
                ax_tpos.cla()
                print(f"Rendering frame {idx}...")

                for j, body in enumerate(self.bodies):
                    # print(f"j={j}, body = {body}, body ID = {body.ID}")
                    # extract plot label
                    if self.body_names[j] is not None:
                        label = self.body_names[j]
                    else:
                        label = None

                    body_pos = self.pos[
                        frames[: idx + 1], 3 * body.ID : 3 * body.ID + 3
                    ]
                    body_time = self.time[frames[: idx + 1]]
                    # print(body_pos)

                    if self.body_colors is not None:
                        color = self.body_colors[j]

                        ax_anim.plot(
                            body_pos[:, 0],
                            body_pos[:, 1],
                            color=color,
                            label=label,
                            alpha=0.1,
                        )
                        ax_anim.scatter(
                            body_pos[-1, 0],
                            body_pos[-1, 1],
                            color=color,
                            s=2,
                            label=label,
                        )

                        # render body
                        body.render_body(ax_anim, body_pos[-1, :], color)

                        # render xyz position
                        ax_xpos.plot(
                            body_time, body_pos[:, 0], color=color, label=label
                        )
                        ax_ypos.plot(
                            body_time, body_pos[:, 1], color=color, label=label
                        )
                        ax_tpos.plot(
                            body_time, body_pos[:, 2], color=color, label=label
                        )

                    else:
                        ax_anim.plot(
                            body_pos[:, 0], body_pos[:, 1], label=label, alpha=0.1
                        )
                        ax_anim.scatter(
                            body_pos[-1, 0], body_pos[-1, 1], s=10, label=label
                        )
                        body.render_body(ax_anim, body_pos[-1, :])
                        ax_xpos.plot(body_time, body_pos[:, 0], label=label)
                        ax_ypos.plot(body_time, body_pos[:, 1], label=label)
                        ax_tpos.plot(body_time, body_pos[:, 2], label=label)

                for j, spring in enumerate(self.springs):
                    if type(spring) == LinearSpringDamper:
                        xp, yp, xc, yc, _, _, _, _ = spring._RelativePos(self.pos[i, :])
                        ax_anim.plot(
                            np.array([xp, xc]), np.array([yp, yc]), color="red"
                        )

                ax_anim.set_xlim(self.xlim[0], self.xlim[1])
                ax_anim.set_ylim(self.ylim[0], self.ylim[1])
                ax_anim.set_title(f"Time = {self.time[i]:.4f}")
                ax_anim.legend()

                ax_xpos.set_xlim(self.timelim[0], self.timelim[1])
                ax_xpos.set_ylim(self.xvlim[0], self.xvlim[1])
                ax_xpos.set_title("X-position")

                ax_ypos.set_xlim(self.timelim[0], self.timelim[1])
                ax_ypos.set_ylim(self.yvlim[0], self.yvlim[1])
                ax_ypos.set_title("Y-position")

                ax_tpos.set_xlim(self.timelim[0], self.timelim[1])
                ax_tpos.set_ylim(self.tvlim[0], self.tvlim[1])
                ax_tpos.set_title("Theta-position")

                writer.grab_frame()
