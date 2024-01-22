
from Agv_planner.STAstar.planner import Planner
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
from DP import dp_planner as dp
from QP import qp_planner as qp
from QP5 import qp_planner as qp5
from OSQP import qp_planner_osqp as qp_osqp
from DP_fun import dp_planner
import math

def main():
    [v_max, v_min, a_max, a_min] = [3.0, 0.0, 2.0, -2.0]
    obs = [(0, 0), (0, 1), (0, 2),    (0, 4), (0, 5), (0, 6),(0, 7),
           (1, 0), (1, 1), (1, 2),    (1, 4), (1, 5), (1, 6),(1, 7),
           (2, 0), (2, 1), (2, 2),    (2, 4), (2, 5), (2, 6),(2, 7),

           (4, 0), (4, 1), (4, 2),    (4, 4), (4, 5), (4, 6),(3, 7),
           (5, 0), (5, 1), (5, 2),    (5, 4), (5, 5), (5, 6),(4, 7),
           (6, 0), (6, 1), (6, 2),    (6, 4), (6, 5), (6, 6),(5, 7),
           (7, 0), (7, 1), (7, 2),    (7, 4), (7, 5), (7, 6),(6, 7),] # +
    STPlanner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=obs)

    # dynamic_obstacles_dict_one = {3: {(3, 3)}, 4: {(3, 3)}}
    dynamic_obstacles_dict_one = dict()
    dynamic_obstacles_dict_two = dict()

    path_one = STPlanner.plan(
        start=(3, 0),
        goal=(3, 6),
        dynamic_obstacles=dynamic_obstacles_dict_one,
        debug=True
    )

    path_two = STPlanner.plan(
        start=(0, 3),
        goal=(6, 3),
        dynamic_obstacles=dynamic_obstacles_dict_two,
        debug=True
    )
    interpolated_path_one = []
    for i in range(len(path_one) - 1):
        interpolated_path_one.extend(np.linspace(path_one[i], path_one[i + 1], num=10, endpoint=False))
    interpolated_path_one = np.array(interpolated_path_one)

    interpolated_path_two = []
    for i in range(len(path_two) - 1):
        interpolated_path_two.extend(np.linspace(path_two[i], path_two[i + 1], num=10, endpoint=False))
    interpolated_path_two = np.array(interpolated_path_two)
    # print(interpolated_path_one)

    # print(path_one[0][0])
    # print(path_one)

    time_points = np.arange(len(path_one))
    displacement_points = []
    dis = 0
    for i in range(len(path_one)):
        if i == 0:
            displacement_points.append(0)
        else:
            dis += np.linalg.norm(path_one[i] - path_one[i - 1])
            displacement_points.append(dis)

    resolution = 1

    interpolated_time = np.arange(time_points[0], time_points[-1] + resolution, resolution)
    interpolated_s = np.arange(displacement_points[0], displacement_points[-1] + resolution, resolution)
    # shape = (len(interpolated_s), len(interpolated_time))# 行数为s，列数为t
    # J_cost2go = np.zeros((shape))
    # state_matrix = np.empty((shape), dtype=object)
    # parent = dict()
    # # state_matrix = [[(0, 0, 0, 0) for _ in range(len(interpolated_time))] for _ in range(len(interpolated_s))]
    # # print(interpolated_time)
    # dp_points_t = []
    # dp_points_s = []
    # start_state = (interpolated_time[0], interpolated_s[0], 0, 0)
    # end_state = (interpolated_time[-1], interpolated_s[-1], 0, 0)
    dy_obs_in = [(2.5, 3.0)]
    dy_obs_out = [(3.5, 3.0)]
    # obs_state = []
    # next_state = ()
    # i = 0
    # best_start_start = None
    # for t in interpolated_time:
    #     j = 0
    #     if t == interpolated_time[0]: # 第一列
    #         state_matrix[j, i] = start_state
    #         parent[start_state] = start_state
    #         i += 1
    #     elif t == interpolated_time[1]: # 第二列
    #         for s in interpolated_s:
    #             next_state = (t, s, 0, 0)
    #             ref_state = (t, interpolated_s[i], 1, 0)
    #             J_cost, next_state = dp(cur_state=start_state, next_state=next_state, end_state=end_state, obs_state=obs_state, ref_state=ref_state, ref_velocity=1, v_max=v_max, v_min=v_min, a_max=a_max, a_min=a_min)
    #             J_cost2go[j, i] = J_cost
    #             state_matrix[j, i] = next_state
    #             parent[next_state] = start_state
    #             j += 1
    #         i += 1
    #     else:
    #         for s in interpolated_s:
    #             J_cost_min = math.inf
    #             k = 0
    #             for cur_state in state_matrix[:, i-1]:
    #                 if cur_state == None:
    #                     continue
    #                 next_state = (t, s, 0, 0)
    #                 if t == interpolated_time[-1]:
    #                     ref_state = end_state
    #                 else:
    #                     ref_state = (t, interpolated_s[i], 0, 0)
    #                 J_cost, next_state = dp(cur_state=cur_state, next_state=next_state, end_state=end_state, obs_state=obs_state, ref_state=ref_state, ref_velocity=1, v_max=v_max, v_min=v_min, a_max=a_max, a_min=a_min)
    #                 J_cost = J_cost + J_cost2go[k, i-1]
    #                 if J_cost < J_cost_min:
    #                     J_cost_min = J_cost
    #                     state_matrix[j, i] = next_state
    #                     best_start_start = cur_state
    #                 k += 1
    #             parent[state_matrix[j, i]] = best_start_start
    #             J_cost2go[j, i] = J_cost_min
    #             j += 1
    #         i += 1

            # plt.plot(best_state[0], best_state[1], marker='o', color='red')
    # # min_index_last_column = np.argmin(J_cost2go[:, -1])
    # # best_state = state_matrix[min_index_last_column, -1]
    # end_state = best_state
    # print(end_state)
    # while True:
    #     dp_points_t.append(best_state[0])
    #     dp_points_s.append(best_state[1])
    #     best_state = parent[best_state]
    #     if best_state == start_state:
    #         dp_points_t.append(best_state[0])
    #         dp_points_s.append(best_state[1])
    #         break
    # dp_points_t.reverse()
    # dp_points_s.reverse()
    # s_lb = []
    # s_ub = []
    #
    # for i in range(len(dp_points_s)):
    #     if dp_points_s[i] - 1 < 0:
    #         s_lb.append(0)
    #     else:
    #         s_lb.append(dp_points_s[i] - 1)
    #
    #     if dp_points_s[i] + 1 > interpolated_s[-1]:
    #         s_ub.append(interpolated_s[-1])
    #     else:
    #         s_ub.append(dp_points_s[i] + 1)
    #
    # print(s_lb, s_ub)

    # s_d_lb = v_min
    # s_d_ub = v_max
    [v_max, v_min, a_max, a_min] = [3.0, 0.0, 2.0, -2.0]
    dp_points_s, dp_points_t = dp_planner(path=path_one, dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out, v_max=v_max, v_min=v_min, a_max=a_max, a_min=a_min)
    # print(dp_points_s)

    # osqp_result = qp_osqp(dp_points_t, dp_points_s, v_max, v_min, start_state=start_state, end_state=end_state, obs_state = obs_state)

    # qp_all_points = qp(dp_points_t, dp_points_s, s_lb, s_ub, s_d_lb, s_d_ub, ref_speed = 1.0, a_max = a_max, a_min = a_min, start_state=start_state, end_state=end_state)
    # print(qp_all_points)
    # qp_points_t = dp_points_t
    # qp_points_s = [qp_all_points[i] for i in range(0, len(qp_all_points), 3)]
    # print(qp_all_points.size, len(qp_points_s), qp_all_points[0], qp_points_s[0])

    # a_all_points = qp5(dp_points_t, dp_points_s, s_lb, s_ub, s_d_lb, s_d_ub, start_state=start_state, end_state=end_state)
    # print(osqp_result.size)
    # a_all_points = np.array(osqp_result)
    # # 将 a_all_points 分成每六个一组
    # coefficients = a_all_points.reshape(-1, 6)
    # # 定义时间范围
    # t = np.linspace(0, 1, 100)
    # # 用于存储所有多项式的值
    # values = []
    # times = np.linspace(0, 6, 600)
    # # 对于每一组系数
    # for coef in coefficients:
    #     # 计算多项式在给定时间范围内的值
    #     value = coef[0] + coef[1] * t + coef[2] * t ** 2 + coef[3] * t ** 3 + coef[4] * t ** 4 + coef[5] * t ** 5
    #     values.append(value)
    #
    # # 将所有的值连接起来
    # values = np.concatenate(values)
    #
    # plt.plot(times, values, color='red')



    # min_indices = np.argmin(J_cost2go, axis=0)
    # min_values = np.min(J_cost2go, axis=0)
    # for i in range(len(min_indices)):
    #     if i == 0:
    #         continue
    #     else:
    #         dp_points_t.append(interpolated_time[i])
    #         dp_points_s.append(interpolated_s[min_indices[i]])
    # print(min_indices, min_values)
    # print(J_cost2go)
    # print(state_matrix)


    # 创建折线图
    for t in interpolated_time:
        t_list = np.full_like(interpolated_s, t)
        plt.scatter(t_list, interpolated_s, marker='o', color='black', alpha=0.2)
    plt.plot(time_points, displacement_points, marker='o', linestyle='-', color='blue')
    plt.plot(dp_points_t, dp_points_s, marker='o', color='purple')
    # plt.plot(qp_points_t, qp_points_s, marker='o', color='purple')
    if len(dy_obs_in) != 0:
        rectangle = Rectangle((3 - 0.5, 3 - 0.5), 1, 1, color='green', alpha=0.5)
        plt.plot(3, 3, marker='o', color='green')
        plt.gca().add_patch(rectangle)

    # 设置图形标签和标题
    plt.xlabel('T')
    plt.ylabel('S')
    plt.title('ST')
    # 图例
    plt.legend(['QP', 'Global Planner', 'DP', 'Obstacle'], loc='upper left')

    # 显示图形
    # plt.savefig('/home/tony/MAPF/Figures/ST.png')
    plt.show()

    need_animation = False

    if need_animation:
        print(interpolated_path_one.shape, interpolated_path_two.shape, values.shape)
        interpolated_path_one_after_plan = [values[i] for i in range(0, len(values), 10)]
        fig, ax = plt.subplots()

        # 设置画面比例
        ax.set_aspect('equal')

        # 设置坐标轴范围
        ax.set_xlim(0, 6)
        ax.set_ylim(0, 6)

        # 创建一个表示方块的矩形
        rect1 = plt.Rectangle((interpolated_path_one[0, 0] - 0.5, interpolated_path_one[0, 1] - 0.5), 1, 1, color='blue', alpha=0.2)
        ax.add_patch(rect1)
        rect2 = plt.Rectangle((interpolated_path_two[0, 0] - 0.5, interpolated_path_two[0, 1] - 0.5), 1, 1, color='green')
        ax.add_patch(rect2)
        rect3 = plt.Rectangle((interpolated_path_one[0, 0] - 0.5, interpolated_path_one_after_plan[0] - 0.5), 1, 1, color='blue')
        ax.add_patch(rect3)

        # 创建表示障碍物的矩形
        for obs_point in obs:
            obstacle_rect = plt.Rectangle((obs_point[0] - 0.5, obs_point[1] - 0.5), 1, 1, color='black', alpha=0.5)
            ax.add_patch(obstacle_rect)

        # 更新函数，用于更新方块的位置
        def update(frame):
            rect1.set_xy((interpolated_path_one[frame, 0] - 0.5, interpolated_path_one[frame, 1] - 0.5))
            rect2.set_xy((interpolated_path_two[frame, 0] - 0.5, interpolated_path_two[frame, 1] - 0.5))
            rect3.set_xy((interpolated_path_one[frame, 0] - 0.5, interpolated_path_one_after_plan[frame] - 0.5))
            return rect1, rect2, rect3

        # 创建动画
        ani = animation.FuncAnimation(fig, update, frames=len(interpolated_path_two), interval=100, blit=True)

        # 在7秒后停止动画
        ani.event_source.stop()
        plt.pause(7)  # 等待7秒钟

        # 保存动画
        # ani.save('animation.gif', writer='imagemagick')  # 你需要安装imagemagick来保存为gif文件
        # ani.save('animation.mp4', writer='ffmpeg')

        # 显示动画
        plt.show()


if __name__ == '__main__':
    main()
