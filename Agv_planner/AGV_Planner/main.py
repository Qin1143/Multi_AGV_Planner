"""
main
@author: Ziqi Qin
"""
import math
from typing import Tuple, List, Dict, Set
import numpy as np
from Global_Planner.global_planner import Planner
from mission import Mission, Env
from plotting import Plotting
from Trajectory_Planner.traj_planner import Traj_Planner
from Trajectory_Planner.DP_fun import dp_planner
from Trajectory_Planner.OSQP import qp_planner_osqp as qp_planner
from tqdm import tqdm
import time


def qp_st_2_xy(path_sub, qp_paths_s, qp_paths_t, corners_index, mission_num):
    """
    将qp的st坐标转化为xy坐标
    """
    final_x = dict()
    final_y = dict()
    for i in range(mission_num):
        for j in range(len(corners_index[i]) // 2 + 1):
            if j == 0:
                point_start = path_sub[(i, j)][0]
                point_end = path_sub[(i, j)][-1]
                # print("point_start", point_start, "point_end", point_end)
                if point_start[0] == point_end[0]:
                    if point_end[1] < point_start[1]:
                        y = np.ones(len(qp_paths_s[(i, j)])) * point_start[1] - qp_paths_s[(i, j)]
                    else:
                        y = np.ones(len(qp_paths_s[(i, j)])) * point_start[1] + qp_paths_s[(i, j)]
                    x = np.ones(len(qp_paths_s[(i, j)])) * point_start[0]
                else:
                    if point_end[0] < point_start[0]:
                        x = np.ones(len(qp_paths_s[(i, j)])) * point_start[0] - qp_paths_s[(i, j)]
                    else:
                        x = np.ones(len(qp_paths_s[(i, j)])) * point_start[0] + qp_paths_s[(i, j)]
                    y = np.ones(len(qp_paths_s[(i, j)])) * point_start[1]
                x = np.append(x, np.ones(100) * x[-1])
                y = np.append(y, np.ones(100) * y[-1])

            elif j == len(corners_index[i]) / 2:
                point_start = path_sub[(i, j)][0]
                point_end = path_sub[(i, j)][-1]
                # print("point_start", point_start, "point_end", point_end)
                if point_start[0] == point_end[0]:
                    if point_end[1] < point_start[1]:
                        y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1] - qp_paths_s[(i, j)])
                    else:
                        y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1] + qp_paths_s[(i, j)])
                    x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0])
                else:
                    if point_end[0] < point_start[0]:
                        x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0] - qp_paths_s[(i, j)])
                    else:
                        x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0] + qp_paths_s[(i, j)])
                    y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1])


            else:
                point_start = path_sub[(i, j)][0]
                point_end = path_sub[(i, j)][-1]
                # print("point_start", point_start, "point_end", point_end)
                if point_start[0] == point_end[0]:
                    if point_end[1] < point_start[1]:
                        y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1] - qp_paths_s[(i, j)])
                    else:
                        y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1] + qp_paths_s[(i, j)])
                    x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0])
                else:
                    if point_end[0] < point_start[0]:
                        x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0] - qp_paths_s[(i, j)])
                    else:
                        x = np.append(x, np.ones(len(qp_paths_s[(i, j)])) * point_start[0] + qp_paths_s[(i, j)])
                    y = np.append(y, np.ones(len(qp_paths_s[(i, j)])) * point_start[1])
                x = np.append(x, np.ones(100) * x[-1])
                y = np.append(y, np.ones(100) * y[-1])

        final_x[i] = x
        final_y[i] = y
    return final_x, final_y



def vertex_cons_2_st(vertex_cons) -> Tuple[List, List]:
    dy_obs_in = []
    dy_obs_out = []
    for index, k in enumerate(vertex_cons[:-1]):  # 遍历vertex_cons，但不包括最后一个元素
        if index == 0 and vertex_cons[index] == 1:
            dy_obs_in.append((index, index))
        elif index == len(vertex_cons) - 2 and vertex_cons[index + 1] == 1:
            dy_obs_out.append((index, index))
        elif k == 0 and vertex_cons[index + 1] == 1:
            dy_obs_in.append((index + 1, index + 1))
        elif k == 1 and vertex_cons[index + 1] == 0:
            dy_obs_out.append((index, index))
        else:
            continue
    # print('dy_obs_in:', dy_obs_in, 'dy_obs_out:', dy_obs_out)
    return dy_obs_in, dy_obs_out


def qp_result2fun(osqp_result, dp_points_t):
    # 将OSqp的结果转化为函数
    qp_coefficient = np.array(osqp_result)
    # 将 a_all_points 分成每六个一组
    coefficient = qp_coefficient.reshape(-1, 6)
    # 定义时间范围
    t = np.linspace(0, 1, 100)
    # 用于存储所有多项式的值
    values = []
    times = np.linspace(0, dp_points_t[-1][0], int(dp_points_t[-1][0]) * 100)
    # 对于每一组系数
    for coef in coefficient:
        # 计算多项式在给定时间范围内的值
        value = coef[0] + coef[1] * t + coef[2] * t ** 2 + coef[3] * t ** 3 + coef[4] * t ** 4 + coef[5] * t ** 5
        values.append(value)

    # 将所有的值连接起来
    values = np.concatenate(values)
    return values, times


def connect_dp_paths(dp_paths_s: dict, dp_paths_t: dict, mission_index: int, corners: dict):
    dp_paths_all = np.array([])
    for j in range(len(corners[mission_index]) // 2 + 1):
        if j == 0:
            DP = np.column_stack((dp_paths_s[(mission_index, j)], dp_paths_t[(mission_index, j)]))
            dp_paths_all = DP
        else:
            DP = np.column_stack((dp_paths_s[(mission_index, j)], dp_paths_t[(mission_index, j)])) + dp_paths_all[-1] + [0, 1]
            dp_paths_all = np.concatenate((dp_paths_all, DP), axis=0)
    return dp_paths_all


def connect_qp_paths(qp_paths_s: dict, qp_paths_t: dict, mission_index: int, corners: dict):
    qp_paths_all = np.array([])
    for j in range(len(corners[mission_index]) // 2 + 1):  # // 代表地板除，忽略小数部分，使用整数部分
        if j == 0:
            qp = np.column_stack((qp_paths_s[(mission_index, j)], qp_paths_t[(mission_index, j)]))
            qp_paths_all = qp
            t = np.linspace(qp_paths_all[-1, 1], qp_paths_all[-1, 1] + 1, 100)
            s = np.ones(100) * qp_paths_all[-1, 0]
            gap = np.column_stack((s, t))
            qp_paths_all = np.concatenate((qp_paths_all, gap), axis=0)
        elif j == len(corners[mission_index]) / 2:
            qp = np.column_stack((qp_paths_s[(mission_index, j)], qp_paths_t[(mission_index, j)])) + qp_paths_all[-1]
            qp_paths_all = np.concatenate((qp_paths_all, qp), axis=0)
        else:
            qp = np.column_stack((qp_paths_s[(mission_index, j)], qp_paths_t[(mission_index, j)])) + qp_paths_all[-1]
            qp_paths_all = np.concatenate((qp_paths_all, qp), axis=0)
            t = np.linspace(qp_paths_all[-1, 1], qp_paths_all[-1, 1] + 1, 100)
            s = np.ones(100) * qp_paths_all[-1, 0]
            gap = np.column_stack((s, t))
            qp_paths_all = np.concatenate((qp_paths_all, gap), axis=0)
    return qp_paths_all

def dynamic_obstacles_dp(dp_paths_all, path, angle, s, dynamic_obstacles):
    # s对应着x, y, angle， 只要将dp_paths_s与s对应起来，就可以得到x, y, angle
    dp_path_xy = [(path[0][0], path[0][1], angle[0])]

    for i in range(1, len(dp_paths_all)):
        index = np.where(s == dp_paths_all[i][0])[0][0]
        dp_path_xy.append((path[index][0], path[index][1], angle[index]))

    for t in range(len(dp_paths_all)):
        if t not in dynamic_obstacles:  # 如果dynamic_obstacles中没有时间层k
            dynamic_obstacles[t] = set()
            dynamic_obstacles[t].add((dp_path_xy[t][0], dp_path_xy[t][1], dp_path_xy[t][2]))
        else:
            dynamic_obstacles[t].add((dp_path_xy[t][0], dp_path_xy[t][1], dp_path_xy[t][2]))

    return dynamic_obstacles


def main():
    # starts = [(25, 80)]
    # goals = [(160, 70)]
    starts = [(5, 5), (20, 15), (10, 70), (15, 35), (15, 20), (15, 10), (10, 15), (25, 80), (15, 15), (20, 55)]
    goals = [(160, 80), (160, 60), (155, 15), (150, 75), (160, 80), (150, 20), (155, 70), (160, 70), (165, 25), (165, 10)]
    env = Env(use_benchmark=True)
    mission = Mission(starts, goals)
    planner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=env.obs)
    paths = dict()
    paths_angle = dict()
    corners_index = dict()
    dynamic_obstacles = dict()
    dp_vertex_constraints = dict()
    up_dp_bound = dict()
    low_dp_bound = dict()

    s = dict()  # 用于存储每个路径的s值
    path_sub = dict()
    dp_paths_s = dict()
    dp_paths_t = dict()
    qp_paths_s = dict()
    qp_paths_t = dict()
    dp_paths_all = dict()
    qp_paths_all = dict()
    dp_vertex_constraints_sub = dict()
    up_dp_bound_sub = dict()
    low_dp_bound_sub = dict()

    [v_max, v_min, a_max, a_min] = [3.0, 0.0, 2.0, -2.0]
    start_time = time.time()
    for i in tqdm(range(mission.mission_num), desc="Motion_Planner"):  # desc确认进度条的名称
        # 在GlobalPlanner中规避edge_constraints，只留下vertex_constraints用于运动规划
        path, path_angle, vertex_constraints, up_bound_list, low_bound_list = planner.plan(
            start=mission.starts[i],
            goal=mission.goals[i],
            dynamic_obstacles=dynamic_obstacles,
            max_iter=1000,
            debug=False
        )
        # print("dynamic_obstacles", dynamic_obstacles)
        # print("up_bound_list", up_bound_list)
        # print("low_bound_list", low_bound_list)
        dp_vertex_constraints[i] = vertex_constraints
        up_dp_bound[i] = up_bound_list
        low_dp_bound[i] = low_bound_list
        # print("mission:", i, "vertex_constraints: ", vertex_constraints)

        # 计算轨迹的拐点
        corner = []
        for j in range(1, len(path) - 1):
            if math.sqrt(
                    math.pow(path[j - 1][0] - path[j + 1][0], 2) + math.pow(path[j - 1][1] - path[j + 1][1], 2)) != 2:
                corner.append(j)
        corners_index[i] = corner
        paths[i] = path
        paths_angle[i] = path_angle

        s[i] = np.zeros(len(paths[i]))
        for k in range(1, len(paths[i])):
            if np.array_equal(paths[i][k], paths[i][k - 1]):
                s[i][k] = s[i][k - 1]
            else:
                s[i][k] = s[i][k - 1] + 1

        for j in range(len(corners_index[i]) // 2 + 1):
            # print("corners_index[i]:", len(corners_index[i]), "j:", j)
            if j == 0:
                path_sub[(i, j)] = paths[i][0:corners_index[i][j] + 1]
                dp_vertex_constraints_sub[(i, j)] = dp_vertex_constraints[i][0:corners_index[i][j] + 1]
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][0:corners_index[i][j] + 1]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][0:corners_index[i][j] + 1]
            elif j == len(corners_index[i]) / 2:
                path_sub[(i, j)] = paths[i][corners_index[i][j * 2 - 1]:]
                dp_vertex_constraints_sub[(i, j)] = dp_vertex_constraints[i][corners_index[i][j * 2 - 1]:]
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][corners_index[i][j * 2 - 1]:]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][corners_index[i][j * 2 - 1]:]
            else:
                path_sub[(i, j)] = paths[i][corners_index[i][j * 2 - 1]:corners_index[i][j * 2] + 1]
                dp_vertex_constraints_sub[(i, j)] = dp_vertex_constraints[i][
                                                    corners_index[i][j * 2 - 1]:corners_index[i][j * 2] + 1]
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][corners_index[i][j * 2 - 1]:corners_index[i][j * 2] + 1]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][corners_index[i][j * 2 - 1]:corners_index[i][j * 2] + 1]

        for j in range(len(corners_index[i]) // 2 + 1):
            if j == 0:  #
                dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])
                dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                               dy_obs_in=dy_obs_in,
                                                                               dy_obs_out=dy_obs_out,
                                                                               v_max=v_max,
                                                                               v_min=v_min,
                                                                               a_max=a_max,
                                                                               a_min=a_min)
                start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)  # t, s, v, a
                end_state = (end_state[0], end_state[1], 0, 0)  # t, s, v, a
                # print("end_state", end_state)
                osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                         start_state=start_state, end_state=end_state, dy_obs_in=[], dy_obs_out=[])  # dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out)
                qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])

            elif j == len(corners_index[i]) / 2:  # 最后一段
                dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])
                dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                               dy_obs_in=dy_obs_in,
                                                                               dy_obs_out=dy_obs_out,
                                                                               v_max=v_max,
                                                                               v_min=v_min,
                                                                               a_max=a_max,
                                                                               a_min=a_min)
                start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)
                end_state = (end_state[0], end_state[1], 0, 0)
                # print("end_state", end_state)
                osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                         start_state=start_state, end_state=end_state, dy_obs_in=[], dy_obs_out=[])
                qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])

            else:  # 中间段
                dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])
                dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                               dy_obs_in=dy_obs_in,
                                                                               dy_obs_out=dy_obs_out,
                                                                               v_max=v_max,
                                                                               v_min=v_min,
                                                                               a_max=a_max,
                                                                               a_min=a_min)
                start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)
                end_state = (end_state[0], end_state[1], 0, 0)
                # print("end_state", end_state)
                osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                         start_state=start_state, end_state=end_state, dy_obs_in=[], dy_obs_out=[])
                qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])
        dp_paths_all[i] = connect_dp_paths(dp_paths_s, dp_paths_t, i, corners_index)  # (s, t)
        qp_paths_all[i] = connect_qp_paths(qp_paths_s, qp_paths_t, i, corners_index)
        dynamic_obstacles = dynamic_obstacles_dp(dp_paths_all[i], path, path_angle, s[i], dynamic_obstacles)

    end_time = time.time()
    print("Time:", end_time - start_time)


    # print("dp_paths_all[4]:", dp_paths_all[4])
    # print("dp_vertex_constraints[4]:", dp_vertex_constraints[4])
    final_x, final_y = qp_st_2_xy(path_sub, qp_paths_s, qp_paths_t, corners_index, mission.mission_num)
    # print("final_y[7]:", final_x[7])
    Plotting(mission.starts, mission.goals, mission.mission_num, env.y_range, env.x_range, env.obs, paths, paths_angle,
             corners_index, dp_paths_all, qp_paths_all, s, up_dp_bound, low_dp_bound, dp_vertex_constraints, final_x, final_y)

if __name__ == '__main__':
    main()

