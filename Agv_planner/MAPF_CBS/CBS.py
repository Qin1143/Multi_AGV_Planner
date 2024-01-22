import math
import matplotlib.pyplot as plt
from CBS.planner import Planner
from CBS.assigner import *
from create_map import get_map_data
import random
import plotting


def main():
    height, width, obstacle_coordinates = get_map_data()
    obs = obstacle_coordinates
    # obs = [(9, 0), (0, 9), (9, 9), (5, 5), (5, 4), (2, 2), (3, 6), (5, 7), (6, 7), (7, 7), (7, 6)]
    # obs = [(0, 0), (0, 1), (0, 2),    (0, 4), (0, 5), (0, 6),
    #        (1, 0), (1, 1), (1, 2),    (1, 4), (1, 5), (1, 6),
    #
    #        (3, 0), (3, 1), (3, 2),    (3, 4), (3, 5), (3, 6),
    #        (4, 0), (4, 1), (4, 2),    (4, 4), (4, 5), (4, 6),] # +
    # print(obs)
    planner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=obs)
    # Starts = [(5, 5), (5, 15), (5, 25)]
    # Goals = [(150, 70), (20, 60), (20, 50)]

    num_iterations = 10
    Starts = []
    Goals = []

    for i in range(num_iterations):
        start_x = random.randint(1, 24)
        start_y = random.randint(1, height-2)
        goal_x = random.randint(width-1-24, width-2)
        goal_y = random.randint(1, height-2)
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        Starts.append(start)
        Goals.append(goal)

    for start_point in Starts:
        if start_point in obs:
            print("start point in obstacle")
            exit(0)

    for goal_point in Goals:
        if goal_point in obs:
            print("goal point in obstacle")
            exit(0)

    # Starts = [(2, 1), (0, 3)] # +
    # Goals = [(3, 3), (2, 5)] # +
    path = planner.plan(
        starts=Starts,
        goals=Goals,
        assign=no_assign,
        max_iter=500,
        low_level_max_iter=2000,
        debug=True
    )

    plot = plotting.Plotting(Starts, Goals)
    plot.multi_plot_2D(path, 'CBS')
    print('Calculate time cost: ', planner.calculate_time)


    # print(path)
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # 绘制静态障碍物
    # for point in obs:
    #     obs_x = point[0]
    #     obs_y = point[1]
    #     obs_z = 0
    #     width = 1
    #     depth = 1
    #     height = len(path[0])
    #
    #     ax.bar3d(obs_x, obs_y, obs_z, width, depth, height, color='black', alpha=0.1)

    # 绘制轨迹立方体
    # for i in range(path.shape[0]):
    #     cube_size = 1
    #     if i == 0:
    #         x = [point[0] for point in path[i]]
    #         y = [point[1] for point in path[i]]
    #         t = [i for i in range(len(path[i]))]
    #         color = 'r'
    #     if i == 1:
    #         x = [point[0] for point in path[i]]
    #         y = [point[1] for point in path[i]]
    #         t = [i for i in range(len(path[i]))]
    #         color = 'g'
    #     if i == 2:
    #         x = [point[0] for point in path[i]]
    #         y = [point[1] for point in path[i]]
    #         t = [i for i in range(len(path[i]))]
    #         color = 'b'
    #
    #     for j in range(len(path[i])):
    #             ax.bar3d(x[j], y[j], t[j], cube_size, cube_size, cube_size, color=color)

    # 设置轴标签
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('T')
    #
    # # 设置 x、y、z 范围
    # ax.set_xlim(0, max([x for x, y in obs])+1)  # 设置 x 范围
    # ax.set_ylim(0, max([y for x, y in obs])+1)  # 设置 y 范围
    # ax.set_zlim(0, len(path[0]))  # 设置 t 范围
    #
    # plt.show()


if __name__ == '__main__':
    main()
