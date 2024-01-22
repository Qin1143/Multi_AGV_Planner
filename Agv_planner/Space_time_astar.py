from STAstar.planner import Planner
import matplotlib.pyplot as plt


def main():
    obs = [(9, 0), (0, 9), (9, 9), (5, 5), (5, 4), (2, 2), (3, 6), (5, 7), (6, 7), (7, 7), (7, 6)]
    # print(obs)
    planner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=obs)
    dynamic_obstacles_dict = {0: {(0, 1)}, 1: {(1, 1)}, 2: {(2, 1)}, 3: {(3, 1)}, 4: {(3, 2), (3, 3), (3, 4)}, 5: {(3, 3), (3, 4)}, 6: {(3, 4)}}
    # print(dynamic_obstacles_dict[1])
    path = planner.plan(
        start=(0, 0),
        goal=(8, 8),
        dynamic_obstacles=dynamic_obstacles_dict,
        debug=True
    )
    print(path)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制静态障碍物
    for point in obs:
        obs_x = point[0]
        obs_y = point[1]
        obs_z = 0
        width = 1
        depth = 1
        height = len(path)

        ax.bar3d(obs_x, obs_y, obs_z, width, depth, height, color='black', alpha=0.1)

    # 绘制动态障碍物
    for key in dynamic_obstacles_dict:
        for point in dynamic_obstacles_dict[key]:
            dobs_x = point[0]
            dobs_y = point[1]
            dobs_z = key
            width = 1
            depth = 1
            height = 1
            ax.bar3d(dobs_x, dobs_y, dobs_z, width, depth, height, color='cyan', alpha=0.4)

    # 绘制轨迹立方体
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    t = [i for i in range(len(path))]
    cube_size = 1
    for i in range(len(path)):
        if i == 0:
            ax.bar3d(x[i], y[i], t[i], cube_size, cube_size, cube_size, color='g')
        elif i == len(path) - 1:
            ax.bar3d(x[i], y[i], t[i], cube_size, cube_size, cube_size, color='b')
        else:
            ax.bar3d(x[i], y[i], t[i], cube_size, cube_size, cube_size, color='r')

    # 设置轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('T')

    # 设置 x、y、z 范围
    ax.set_xlim(0, 10)  # 设置 x 范围
    ax.set_ylim(0, 10)  # 设置 y 范围
    ax.set_zlim(0, len(path))  # 设置 t 范围

    plt.show()


if __name__ == '__main__':
    main()
