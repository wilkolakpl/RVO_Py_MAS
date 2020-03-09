import numpy as np

from Robot import Robot
from vis import Visualizer

X = 0
Y = 1
YAW = 2
ROBOT_RADIUS = 0.2

circular_obstacles = []
# circular_obstacles = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]

poses = [[-0.5+1.0*i, 0.0, np.pi/2] for i in range(7)] + \
        [[-0.5+1.0*i, 5.0, -np.pi/2] for i in range(7)]

goals = [[5.5-1.0*i, 5.0] for i in range(7)] + \
        [[5.5-1.0*i, 0.0] for i in range(7)]

poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2],
         [0.0, 2.5, 0], [5.0, 2.5, np.pi]]

goals = [[2.5, 5.0], [2.5, 0.0], [5.0, 2.5], [0.0, 2.5]]

# poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2]]

# goals = [[2.5, 5.0], [2.5, 0.0]]

robots = []
for i, (pose, goal) in enumerate(zip(poses, goals)):
    if i == 0:
        mischevious = True
    else:
        mischevious = False
    robots.append(Robot(pose, goal, robot_radius=ROBOT_RADIUS,
                        mischevious=mischevious))


circular_obstacles = [[-0.3, 2.5, 0.3],
                      [1.5, 2.5, 0.3],
                      [3.3, 2.5, 0.3],
                      [5.1, 2.5, 0.3]]
viz = Visualizer(robot_radius=ROBOT_RADIUS, circular_obstacles=[])

total_time = 15
step = 0.01
t = 0
while t*step < total_time:

    for robot in robots:
        robot.update_RVO_velocity(robots, circular_obstacles)

    for robot in robots:
        robot.update_pose(step)

    for robot in robots:
        robot.exchange_velocities(robots, step)

    if t % 20 == 0:
        viz.visualize(robots, time=t*step)
    # name='data/snap%s.png' % str(t/10))

    t += 1
