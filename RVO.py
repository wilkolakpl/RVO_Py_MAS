from math import ceil, floor, sqrt
import copy
import numpy

from math import cos, sin, tan, atan2, asin

from math import pi as PI


X = 0
Y = 1
YAW = 2


def calc_distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001


def is_in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False


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


def RVO_update(robots, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius']+0.1

    for robot in robots:
        RVO_BA_all = []
        for other_robot in robots:
            if robot != other_robot:
                RVO_BA = calc_RVO(robot.pose[:2], other_robot.pose[:2],
                                  robot.velocity, other_robot.velocity,
                                  ROB_RAD)
                RVO_BA_all.append(RVO_BA)
        # for hole in ws_model['circular_obstacles']:
        #     # hole = [x, y, rad]
        #     vB = [0, 0]
        #     pB = hole[0:2]
        #     transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
        #     dist_BA = calc_distance(pA, pB)
        #     theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
        #     # over-approximation of square to circular
        #     OVER_APPROX_C2S = 1.5
        #     rad = hole[2]*OVER_APPROX_C2S
        #     if (rad+ROB_RAD) > dist_BA:
        #         dist_BA = rad+ROB_RAD
        #     theta_BAort = asin((rad+ROB_RAD)/dist_BA)
        #     theta_ort_left = theta_BA+theta_BAort
        #     bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
        #     theta_ort_right = theta_BA-theta_BAort
        #     bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
        #     RVO_BA = [transl_vB_vA, bound_left,
        #               bound_right, dist_BA, rad+ROB_RAD]
        #     RVO_BA_all.append(RVO_BA)
        vA_post = intersect(robot.pose[:2], robot.compute_V_des(), RVO_BA_all)
        robot.velocity = vA_post[:]


def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersection test'
    norm_v = calc_distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2*PI, 0.1):
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                theta_left = RVO_BA[1]
                theta_right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                if is_in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        theta_left = RVO_BA[1]
        theta_right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        if is_in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    # ----------------------
    if suitable_V:
        # print 'Suitable found'
        vA_post = min(suitable_V, key=lambda v: calc_distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            theta_left = RVO_BA[1]
            theta_right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                theta_left = RVO_BA[1]
                theta_right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                if is_in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta)) - \
                        abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0
                    tc_v = dist_tg/calc_distance(dif, [0, 0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key=lambda v: (
            (WT/tc_V[tuple(v)])+calc_distance(v, vA)))
    return vA_post
