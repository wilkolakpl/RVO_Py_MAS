#!/usr/bin/env python
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np

from math import pi as PI
from math import atan2, sin, cos, sqrt

plt.ion()

X = 0
Y = 1
YAW = 2


class Visualizer:
    def __init__(self, robot_radius=0.2, circular_obstacles=[]):
        self.robot_radius = robot_radius
        self.circular_obstacles = circular_obstacles

        self.figure = None
        self.ax = None

    def _get_cmap(self, N):
        '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
        color_norm = colors.Normalize(vmin=0, vmax=N)
        scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

        def map_index_to_rgb_color(index):
            return scalar_map.to_rgba(index)
        return map_index_to_rgb_color

    def initialize(self, robots):
        self.figure = plt.figure(figsize=(8, 8))
        plt.show(block=False)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1.0, 6.0)
        self.ax.set_ylim(-1.0, 6.0)
        self.ax.set_xlabel(r'$x (m)$')
        self.ax.set_ylabel(r'$y (m)$')
        self.ax.grid(True)

        self.cmap = self._get_cmap(len(robots))

        self.clock = self.ax.text(2, 5.5, '$t=%.1f s$' % 0,
                                  fontsize=20, fontweight='bold')

        self.robot_pts = []
        self.history_x = []
        self.history_y = []
        self.path = []
        self.labels = []
        self.arrows = []
        for i, robot in enumerate(robots):
            # PLOT ROBOTS
            robot_pt = patches.Wedge(
                (robot._pose[X], robot._pose[Y]),
                self.robot_radius,
                np.degrees(robot._pose[YAW]) + 30,
                np.degrees(robot._pose[YAW]) - 30,
                facecolor=self.cmap(i),
                edgecolor='black',
                linewidth=1.0,
                ls='solid',
                alpha=1,
                zorder=2)
            self.ax.add_patch(robot_pt)
            self.robot_pts.append(robot_pt)

            # PLOT ROBOT PATHS
            self.path.append(self.ax.plot([], [], color=self.cmap(i))[0])
            self.history_x.append([])
            self.history_y.append([])
            self.history_x[i].append(robot._pose[X])
            self.history_y[i].append(robot._pose[Y])
            self.path[i].set_data(self.history_x[i], self.history_y[i])

            # PLOT ROBOT LABELS
            label = self.ax.text(robot._pose[X]-0.1, robot._pose[Y]-0.1, r'$%s$' %
                                 i, fontsize=15, fontweight='bold', zorder=3)
            self.labels.append(label)

            # PLOT ROBOT VELOCITIES
            arrowstyle = patches.ArrowStyle.Simple(
                head_length=0.2,
                head_width=0.2,
                tail_width=0.1)
            arrow = patches.FancyArrowPatch(
                (robot._pose[X], robot._pose[Y]),
                (robot._pose[X]+robot._velocity[X],
                 robot._pose[Y]+robot._velocity[Y]),
                color=self.cmap(i))
            arrow.set_arrowstyle("fancy", head_length=10, head_width=5)
            self.ax.add_patch(arrow)
            self.arrows.append(arrow)

            # PLOT ROBOT GOALS
            self.ax.plot(
                [robot._goal[X]],
                [robot._goal[Y]],
                'x',
                color=self.cmap(i),
                markersize=15)

        # PLOT OBSTACLES
        for hole in self.circular_obstacles:
            srec = patches.Rectangle(
                (hole[0]-hole[2], hole[1]-hole[2]),
                2*hole[2], 2*hole[2],
                facecolor='red',
                fill=True,
                alpha=1)
            self.ax.add_patch(srec)

    def visualize(self, robots, time=None, name=None):
        if not self.figure:
            self.initialize(robots)

        for i, robot in enumerate(robots):
            # PLOT ROBOTS
            self.robot_pts[i].set_center((robot._pose[X], robot._pose[Y]))
            self.robot_pts[i].set_theta1(np.degrees(robot._pose[YAW]) + 30)
            self.robot_pts[i].set_theta2(np.degrees(robot._pose[YAW]) - 30)

            # PLOT ROBOT PATHS
            self.history_x[i].append(robot._pose[X])
            self.history_y[i].append(robot._pose[Y])
            self.path[i].set_data(self.history_x[i], self.history_y[i])

            # PLOT ROBOT LABELS
            self.labels[i].set_position(
                (robot._pose[X]-0.1, robot._pose[Y]-0.1))

            # PLOT ROBOT VELOCITIES
            self.arrows[i].set_positions(
                (robot._pose[X], robot._pose[Y]),
                (robot._pose[X]+robot._velocity[X],
                 robot._pose[Y]+robot._velocity[Y])
            )

        # SHOW CURRENT TIME
        if time:
            self.clock.set_text('$t=%.1f s$' % time)

        self.figure.canvas.draw_idle()
        plt.pause(0.0001)

        if name:
            plt.savefig(name, dpi=200)
