import numpy as np
import click

from Robot import Robot
from vis import Visualizer

X = 0
Y = 1
YAW = 2
ROBOT_RADIUS = 0.2


def prepare_poses_and_goals(scenario, number=7):
    if scenario == "head_on":
        poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2]]
        goals = [[2.5, 5.0], [2.5, 0.0]]

    elif scenario == "4_way_crossing":
        poses = [[2.5, 0.0, np.pi/2], [2.5, 5.0, -np.pi/2],
                 [0.0, 2.5, 0], [5.0, 2.5, np.pi]]
        goals = [[2.5, 5.0], [2.5, 0.0], [5.0, 2.5], [0.0, 2.5]]

    elif scenario == "moshpit":
        poses = [[-0.5+1.0*i, 0.0, np.pi/2] for i in range(number)] + \
                [[-0.5+1.0*i, 5.0, -np.pi/2] for i in range(number)]

        goals = [[5.5-1.0*i, 5.0] for i in range(number)] + \
                [[5.5-1.0*i, 0.0] for i in range(number)]

    elif scenario == "blocking":
        poses = [[2., 1.8, np.pi/2], [3., 3.2, -np.pi/2],
                 [1.8, 3., 0], [3.2, 2., np.pi], [2.5, 2.5, np.pi]]
        goals = [[2., 1.8], [3., 3.2], [1.8, 3.], [3.2, 2.], [5., 5.]]

    return poses, goals


def prepare_robots(scenario, controller, number=7):
    poses, goals = prepare_poses_and_goals(scenario, number)

    robots = []
    for i, (pose, goal) in enumerate(zip(poses, goals)):

        if scenario == "blocking":
            selfish = False
            blocking = True if i < 4 else False
            vel_det_method = controller
        elif controller == "honest":
            selfish = False
            blocking = False
            vel_det_method = "broadcast"
        else:
            selfish = True if i == 0 else False
            blocking = False
            vel_det_method = controller

        robots.append(Robot(pose, goal, robot_radius=ROBOT_RADIUS,
                            selfish=selfish, vel_det_method=vel_det_method,
                            blocking=blocking, red=(i == 0)))
    return robots


def prep_timers(scenario):
    if scenario != "blocking":
        total_flow_time = 0
        red_flow_time = 0
    else:
        total_flow_time = None
        red_flow_time = None
    return [total_flow_time, red_flow_time]


def update_timers(robots, scenario, timers, step):
    if scenario != "blocking":
        for robot in robots:
            if not robot.reached_goal():
                timers[0] += step
                if robot.red:
                    timers[1] += step
    total_flow_time = timers[0]
    total_flow_time = None if total_flow_time is None \
        else total_flow_time / len(robots)
    red_flow_time = timers[1]
    return total_flow_time, red_flow_time


def run_loop(scenario, controller, robots, visualize=True):
    viz = Visualizer(robot_radius=ROBOT_RADIUS)
    choppy = True

    timers = prep_timers(scenario)

    total_time = 10
    if scenario == "moshpit":
        total_time += 5

    step = 0.01
    t = 0
    while t*step < total_time:

        (total_flow_time,
         red_flow_time) = update_timers(robots, scenario, timers, step)

        for robot in robots:
            robot.update(robots, step)
        for robot in robots:
            robot.exchange_velocities(robots, step)

        if visualize:
            if t % 10 != 0 and choppy:
                pass
            else:
                viz.visualize(robots, time=t, step=step,
                              total_flow_time=total_flow_time,
                              red_flow_time=red_flow_time,
                              name='data/' + controller + '_' + scenario + '/snap')

        t += 1

    return total_flow_time, red_flow_time


def generate_breakdown():
    for i in range(1, 8):
        robots = prepare_robots("moshpit", "broadcast", i)
        tft_b, rft_b = run_loop("moshpit", "broadcast",
                                robots, visualize=False)
        robots = prepare_robots("moshpit", "honest", i)
        tft_h, rft_h = run_loop("moshpit", "honest",
                                robots, visualize=False)
        robots = prepare_robots("moshpit", "inference", i)
        tft_i, rft_i = run_loop("moshpit", "inference",
                                robots, visualize=False)
        print(i, "honest,t,r", tft_h, rft_h, "broadcast,t,r",
              tft_b, rft_b, "inference,t,r", tft_i, rft_i)


@click.command()
@click.option("--controller", type=str, default="broadcast")
@click.option("--scenario", type=str, default="4_way_crossing")
@click.option("--gen_breakdown", type=bool, default=False)
def run(controller, scenario, gen_breakdown):
    if gen_breakdown:
        generate_breakdown()
    else:
        robots = prepare_robots(scenario, controller)
        run_loop(scenario, controller, robots)


if __name__ == "__main__":
    run()
