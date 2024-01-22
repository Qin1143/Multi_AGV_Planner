"""
AGV_Planner
@author: Ziqi Qin
"""
import math

import numpy as np
from Global_Planner.global_planner import Planner
from mission import Mission, Env
from plotting import Plotting
from Trajectory_Planner.traj_planner import Traj_Planner


def main():
    starts = [(5, 5), (20, 15)]
    goals = [(160, 80), (160, 60)]
    env = Env(use_benchmark=True)
    mission = Mission(starts, goals)
    planner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=env.obs)
    paths = dict()
    corners_index = dict()
    # planner并未对目标点方向进行约束
    for i in range(mission.mission_num):
        path = planner.plan(
            start=mission.starts[i],
            goal=mission.goals[i],
            dynamic_obstacles=dict(),
            max_iter=1000,
            debug=True
        )
        corner = []
        for j in range(1,len(path)-1):
            if math.sqrt(math.pow(path[j-1][0] - path[j+1][0], 2) + math.pow(path[j-1][1] - path[j+1][1], 2)) != 2:
                corner.append(j)
        corners_index[i] = corner
        paths[i] = path

    print(corners_index)

    Plotting(mission.starts, mission.goals, mission.mission_num, env.y_range, env.x_range, env.obs, paths, corners_index,
             'Multi_AGV_Planner')
    traj_planner = Traj_Planner(paths, dict(), mission.mission_num, corners_index)


if __name__ == '__main__':
    main()
