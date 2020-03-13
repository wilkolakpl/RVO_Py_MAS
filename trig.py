import numpy as np
from math import pi, atan2, asin, sin, cos

X = 0
Y = 1
YAW = 2


def calc_distance(pose1, pose2):
    diff = np.array(pose1) - np.array(pose2)
    return np.linalg.norm(diff) + 0.001


def is_in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= pi:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2*pi
            if theta_dif < 0:
                theta_dif += 2*pi
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2*pi
            if theta_dif < 0:
                theta_dif += 2*pi
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False


def calc_RVO_hole(posA, velA, hole, rob_rad):
    velB = [0, 0]
    posB = hole[0:2]

    center = [posA[0]+velB[0], posA[1]+velB[1]]

    dist_BA = calc_distance(posA, posB)
    theta_BA = atan2(posB[1]-posA[1], posB[0]-posA[0])

    # over-approximation of square to circular
    OVER_APPROX_C2S = 1.5
    rad = hole[2]*OVER_APPROX_C2S
    if (rad+rob_rad) > dist_BA:
        dist_BA = rad+rob_rad

    theta_BAort = asin((rad+rob_rad)/dist_BA)
    left_angle = theta_BA + theta_BAort
    right_angle = theta_BA - theta_BAort

    rad = rad + rob_rad

    return center, left_angle, right_angle, dist_BA, rad


def calc_RVO(posA, posB, velA, velB, rob_rad):
    center = [posA[X] + 0.5 * (velB[X]+velA[X]),
              posA[Y] + 0.5 * (velB[Y]+velA[Y])]

    dist_BA = calc_distance(posA, posB)

    theta_BA = atan2(posB[Y] - posA[Y], posB[X] - posA[X])
    if 2*rob_rad > dist_BA:
        dist_BA = 2 * rob_rad
    theta_BAort = asin(2 * rob_rad / dist_BA)
    left_angle = theta_BA + theta_BAort
    right_angle = theta_BA - theta_BAort

    rad = 2*rob_rad

    return center, left_angle, right_angle, dist_BA, rad


def verify_vel_outside_obstacles(posA, new_velA, RVO_BA_all):
    for RVO_BA in RVO_BA_all:
        center = RVO_BA[0]
        theta_left = RVO_BA[1]
        theta_right = RVO_BA[2]
        dif = [new_velA[X] + posA[X] - center[X],
               new_velA[Y] + posA[Y] - center[Y]]
        theta_dif = atan2(dif[Y], dif[X])
        if is_in_between(theta_right, theta_dif, theta_left):
            distance = RVO_BA[3]
            rad = RVO_BA[4]
            angle = theta_dif-0.5*(theta_left+theta_right)
            prop = 0.4
            if np.linalg.norm(dif)*cos(angle) > prop*(distance - rad/2):
                return False
    return True


def compute_distance_to_cone_edge(theta_right, theta_dif,
                                  theta_left, rad, dist, dif):
    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))

    if abs(dist*sin(small_theta)) >= rad:
        rad = abs(dist*sin(small_theta))

    big_theta = asin(abs(dist*sin(small_theta))/rad)
    dist_tg = abs(dist*cos(small_theta)) - abs(rad*cos(big_theta))

    if dist_tg < 0:
        dist_tg = 0

    return dist_tg/calc_distance(dif, [0, 0])
