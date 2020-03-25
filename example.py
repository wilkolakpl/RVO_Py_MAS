import numpy as np

from Robot import Robot
from vis import Visualizer

X = 0
Y = 1
YAW = 2
ROBOT_RADIUS = 0.2

can_cheat = True
all_honest = True
scenario = "moshpit"

if can_cheat:
    vel_det_method = "broadcast"
else:
    vel_det_method = "inference"

if scenario == "head_on":
    poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2]]
    goals = [[2.5, 5.0], [2.5, 0.0]]

elif scenario == "4_way_crossing":
    poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2],
             [0.0, 2.5, 0], [5.0, 2.5, np.pi]]
    goals = [[2.5, 5.0], [2.5, 0.0], [5.0, 2.5], [0.0, 2.5]]

elif scenario == "moshpit":
    poses = [[-0.5+1.0*i, 0.0, np.pi/2] for i in range(7)] + \
            [[-0.501+1.0*i, 5.0, -np.pi/2] for i in range(7)]

    goals = [[5.5-1.0*i, 5.0] for i in range(7)] + \
            [[5.5-1.0*i, 0.0] for i in range(7)]

elif scenario == "blocking":
    poses = [[2., 1.8, np.pi/2], [3., 3.2, -np.pi/2],
             [1.8, 3., 0], [3.2, 2., np.pi], [2.5, 2.5, np.pi]]
    goals = [[2., 1.8], [3., 3.2], [1.8, 3.], [3.2, 2.], [5., 5.]]


def prepare_robots():
    robots = []
    for i, (pose, goal) in enumerate(zip(poses, goals)):
        if all_honest:
            selfish = False
            blocking = False
        elif scenario == "blocking":
            selfish = False
            blocking = True if i < 4 else False
        else:
            selfish = True if i == 0 else False
            blocking = False

        robots.append(Robot(pose, goal, robot_radius=ROBOT_RADIUS,
                            selfish=selfish, vel_det_method=vel_det_method, blocking=blocking))
    return robots


def run_loop(robots):
    viz = Visualizer(robot_radius=ROBOT_RADIUS)

    choppy = False

    total_time = 15
    step = 0.01
    t = 0
    while t*step < total_time:

        for robot in robots:
            robot.update_RVO_velocity(robots)

        for robot in robots:
            robot.update_pose(step)

        for robot in robots:
            robot.exchange_velocities(robots, step)

        if t % 10 != 0 and choppy:
            pass
        else:
            viz.visualize(robots, time=t*step,
                          name='data/snap%s.png' % str(t))

        t += 1


if __name__ == "__main__":
    robots = prepare_robots()
    run_loop(robots)
