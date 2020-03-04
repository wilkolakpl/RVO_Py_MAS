import sys
import numpy as np

from RVO import RVO_update, calc_distance
from vis import Visualizer

X = 0
Y = 1
YAW = 2

# ------------------------------
# define workspace model
ws_model = dict()
# robot radius
ws_model['robot_radius'] = 0.2
# circular obstacles, format [x,y,rad]
# no obstacles
ws_model['circular_obstacles'] = []
# with obstacles
# ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
# rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = []

# ------------------------------
# initialization for robot
# position of [x,y]
pos = [[-0.5+1.0*i, 0.0]
       for i in range(1)] + [[-0.5+1.0*i, 5.0] for i in range(1)]

theta = [np.pi/2 for i in range(1)] + [-np.pi/2 for i in range(1)]

theta_max = [np.pi/32 for i in range(len(pos))]
# velocity of [vx,vy]
V = [[0, 0] for i in range(len(pos))]
# maximal velocity norm
V_max = [1.0 for i in range(len(pos))]
# goal of [x,y]
goal = [[5.5-1.0*i, 5.0]
        for i in range(1)] + [[5.5-1.0*i, 0.0] for i in range(1)]


class Robot:
    def __init__(self, pose, velocity, goal, v_max, theta_max):
        self.pose = pose
        self.velocity = velocity
        self.goal = goal
        self.v_max = v_max
        self.theta_max = theta_max

    def compute_V_des(self):
        if self.reached_goal(0.1):
            return [0, 0]

        dif_x = [self.goal[X]-self.pose[X],
                 self.goal[Y]-self.pose[Y]]
        norm = calc_distance(dif_x, [0, 0])
        return [dif_x[X] * self.v_max / norm,
                dif_x[Y] * self.v_max / norm]

    def reached_goal(self, bound=0.5):
        if calc_distance(self.pose[:2], self.goal) < bound:
            return True
        else:
            return False


robots = []
for i in range(len(pos)):
    robots.append(Robot([pos[i][X], pos[i][Y], theta[i]],
                        V[i], goal[i], V_max[i], theta_max[i]))

# ------------------------------
# simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.01
# setup visualization
viz = Visualizer()

# ------------------------------
# simulation starts
t = 0
while t*step < total_time:

    RVO_update(robots, ws_model)

    for i, robot in enumerate(robots):

        # Respect the non-holonomic constraints using http://www.alonsomora.com/docs/10-alonsomora.pdf
        # theta_H = np.arctan2(V[i][Y], V[i][X]) - robot.pose[YAW]
        # V_H = np.linalg.norm(V[i])
        # v_star = V_H * theta_H * np.sin(theta_H)
        # v_star /= 2 * (1 - np.cos(theta_H))

        # omega = theta_H
        # if abs(omega) <= robot.theta_max and v_star <= robot.v_max:
        #     pass
        # elif abs(omega) <= robot.theta_max and v_star > robot.v_max:
        #     v_star = robot.v_max
        # else:
        #     v_star = 0
        #     if omega > 0:
        #         omega = robot.theta_max
        #     else:
        #         omega = -robot.theta_max

        # robot.pose[YAW] += omega

        # V[i][X] = v_star * np.cos(robot.pose[YAW])
        # V[i][Y] = v_star * np.sin(robot.pose[YAW])

        robot.pose[X] += robot.velocity[X] * step
        robot.pose[Y] += robot.velocity[Y] * step

        # while robot.pose[YAW] < -np.pi:
        #     robot.pose[YAW] += 2*np.pi
        # while robot.pose[YAW] > np.pi:
        #     robot.pose[YAW] -= 2*np.pi
    # ----------------------------------------
    # visualization
    if t % 10 != 0:
        # name='data/snap%s.png' % str(t/10))
        viz.visualize(ws_model, robots, time=t*step)
    t += 1
