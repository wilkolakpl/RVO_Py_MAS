import numpy as np
from math import cos, sin, atan2, pi
from collections import defaultdict
from trig import calc_distance, is_in_between, calc_RVO, \
    verify_vel_outside_obstacles, compute_distance_to_cone_edge


X = 0
Y = 1
YAW = 2

DETECTION_DISTANCE = 3


class Robot:
    def __init__(self, pose, goal,
                 velocity=[0, 0],
                 reversing=False,
                 v_max=1,
                 theta_max=pi/16,
                 robot_radius=0.2,
                 selfish=False,
                 blocking=False,
                 vel_det_method="broadcast"):

        self._pose = pose
        self._velocity = velocity
        self._reversing = reversing
        self._vel_det_method = vel_det_method

        self._v_max = v_max
        self._theta_max = theta_max
        self._radius = robot_radius

        self._goal = goal

        self._selfish = selfish
        self._blocking = blocking
        self._neighbor_vels = defaultdict(lambda: [0., 0.])
        self._neighbor_poses = defaultdict(lambda: [0., 0.])

    def update_pose(self, step):
        self._pose[X] += self._velocity[X] * step
        self._pose[Y] += self._velocity[Y] * step

        if self._reached_goal():
            return

        new_YAW = atan2(self._velocity[Y], self._velocity[X])
        if self._reversing:
            self._pose[YAW] = new_YAW + pi
        else:
            self._pose[YAW] = new_YAW

    def exchange_velocities(self, robots, step):
        if self._vel_det_method == "broadcast":
            for neighbor in robots:
                if calc_distance(self._pose[:2],
                                 neighbor._pose[:2]) < DETECTION_DISTANCE:

                    if self._blocking:
                        towards_goal = self._compute_optimal_V()
                        neighbor._neighbor_vels[self] = [
                            self._v_max * cos(self._pose[YAW]), self._v_max * sin(self._pose[YAW])]

                    elif self._selfish:
                        towards_other = [neighbor._pose[X] - self._pose[X],
                                         neighbor._pose[Y] - self._pose[Y]]
                        other_angle = atan2(towards_other[Y], towards_other[X])
                        towards_goal = self._compute_optimal_V()
                        goal_angle = atan2(towards_goal[Y], towards_goal[X])
                        angle = 0.5*(other_angle + goal_angle)
                        neighbor._neighbor_vels[self] = [
                            self._v_max * cos(angle), self._v_max * sin(angle)]
                    else:
                        neighbor._neighbor_vels[self] = self._velocity

        elif self._vel_det_method == "inference":
            for neighbor in robots:
                if calc_distance(self._pose[:2],
                                 neighbor._pose[:2]) < DETECTION_DISTANCE:
                    past_pose = self._neighbor_poses[neighbor]
                    curr_pose = neighbor._pose[:2]
                    self._neighbor_vels[neighbor] = np.array(
                        curr_pose) - np.array(past_pose)
                    self._neighbor_vels[neighbor] = self._neighbor_vels[neighbor] / step

                    self._neighbor_poses[neighbor] = curr_pose

    def update_RVO_velocity(self, robots):

        ROB_RAD = self._radius + 0.1
        RVO_BA_all = []
        for other_robot in robots:
            if self != other_robot and calc_distance(self._pose[:2],
                                                     other_robot._pose[:2]) < DETECTION_DISTANCE:
                RVO_BA = calc_RVO(
                    self._pose[:2], other_robot._pose[:2],
                    self._velocity, self._neighbor_vels[other_robot],
                    ROB_RAD
                )
                RVO_BA_all.append(RVO_BA)

        self._velocity, self._reversing = self._intersect(RVO_BA_all)

    def _intersect(self, RVO_BA_all):
        optimal_V = self._compute_optimal_V()
        norm_v = calc_distance(optimal_V, [0, 0])
        norm_curr = calc_distance(self._velocity, [0, 0])
        suitable_V = []
        unsuitable_V = []

        increments = np.arange(0, self._theta_max, 0.01)
        thetas = np.concatenate((-increments[1:] + self._pose[YAW],
                                 increments + self._pose[YAW]))

        for theta in thetas:
            for rad in np.arange(max(-norm_curr - norm_v / 4, -norm_v),
                                 min(norm_curr + norm_v / 4, norm_v), norm_v/10.0):
                new_v = [rad*cos(theta), rad*sin(theta)]
                reversing = False if rad > 0 else True

                if verify_vel_outside_obstacles(self._pose[:2], new_v, RVO_BA_all):
                    suitable_V.append((new_v, reversing))
                else:
                    unsuitable_V.append((new_v, reversing))

        if suitable_V:
            velA_updated = min(
                suitable_V, key=lambda v: calc_distance(v[0], optimal_V))
        else:
            tc_V = dict()
            for unsuit_v, _ in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for RVO_BA in RVO_BA_all:
                    center = RVO_BA[0]
                    theta_left, theta_right = RVO_BA[1], RVO_BA[2]
                    dist, rad = RVO_BA[3], RVO_BA[4]

                    dif = [unsuit_v[X] + self._pose[X] - center[X],
                           unsuit_v[Y] + self._pose[Y] - center[Y]]
                    theta_dif = atan2(dif[1], dif[0])

                    if is_in_between(theta_right, theta_dif, theta_left):
                        tc_v = compute_distance_to_cone_edge(theta_right, theta_dif,
                                                             theta_left, rad, dist, dif)
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc)+0.001
            velA_updated = min(unsuitable_V, key=lambda v:
                               20 / tc_V[tuple(v[0])] +
                               calc_distance(v[0], optimal_V))
        return velA_updated

    def _compute_optimal_V(self):
        if self._reached_goal(0.1):
            return [0, 0]

        dif_x = [self._goal[X]-self._pose[X],
                 self._goal[Y]-self._pose[Y]]
        norm = calc_distance(dif_x, [0, 0])
        return [dif_x[X] * self._v_max / norm,
                dif_x[Y] * self._v_max / norm]

    def _reached_goal(self, bound=0.5):
        if calc_distance(self._pose[:2], self._goal) < bound:
            return True
        else:
            return False
