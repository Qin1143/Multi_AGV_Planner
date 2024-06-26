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
import pandas as pd


def qp_st_2_xy(path_sub, qp_paths_s, qp_paths_t, corners, mission_num):
    """
    将qp的st坐标转化为xy坐标
    """
    final_x = dict()
    final_y = dict()
    for i in range(mission_num):
        for j in range(len(corners[i]) + 1):
            if j == 0:
                point_start = path_sub[(i, j)][0]
                point_end = path_sub[(i, j)][-1]
                # print("point_start", point_start, "point_end", point_end)
                if point_start[0] == point_end[0] and point_start[1] == point_end[1]:  # 如果起点和终点重合,等待段
                    x = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0]
                    y = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1]
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape)
                    continue
                if point_start[0] == point_end[0]:  # 如果x坐标相等, 说明是竖直方向移动
                    if point_end[1] < point_start[1]:  # 向下移动
                        y = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1] - qp_paths_s[(i, j)]
                    else:  # 向上移动
                        y = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1] + qp_paths_s[(i, j)]
                    x = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0]
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape)
                else:   # 如果y坐标相等, 说明是水平方向移动
                    if point_end[0] < point_start[0]:  # 向左移动
                        x = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0] - qp_paths_s[(i, j)]
                    else:  # 向右移动
                        x = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0] + qp_paths_s[(i, j)]
                    y = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1]
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape)
            else:
                point_start = path_sub[(i, j)][0]
                point_end = path_sub[(i, j)][-1]
                # print("point_start", point_start, "point_end", point_end)
                if point_start[0] == point_end[0] and point_start[1] == point_end[1]:  # 如果起点和终点重合,等待段
                    x_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0]
                    y_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1]
                    x = np.append(x, x_sub)
                    y = np.append(y, y_sub)
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape, "len_x_sub", x_sub.shape, "len_y_sub", y_sub.shape)
                    continue
                if point_start[0] == point_end[0]:
                    if point_end[1] < point_start[1]:
                        y_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1] - qp_paths_s[(i, j)]
                        y = np.append(y, y_sub)
                    else:
                        y_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1] + qp_paths_s[(i, j)]
                        y = np.append(y, y_sub)
                    x = np.append(x, np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0])
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape, "len_y_sub:", y_sub.shape)
                else:
                    if point_end[0] < point_start[0]:
                        x_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0] - qp_paths_s[(i, j)]
                        x = np.append(x, x_sub)
                    else:
                        x_sub = np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[0] + qp_paths_s[(i, j)]
                        x = np.append(x, x_sub)
                    y = np.append(y, np.ones( (len(qp_paths_s[(i, j)]), 1) ) * point_start[1])
                    # print("len_x:", x.shape, "len_y:", y.shape, "len_qp_paths_s:", qp_paths_s[(i, j)].shape, "len_x_sub:", x_sub.shape)

        final_x[i] = x
        final_y[i] = y
        # print("Mission:", i, "final_x:", len(final_x[i]), "final_Y：", len(final_y[i]))
    return final_x, final_y

def s_2_v(s, x, y, mission_num):
    """
    将s数据转化为v数据，v = ds/dt，并且储存在对应的excel文件中
    """
    dt = 0.01
    df_total = pd.DataFrame()

    for i in range(mission_num):
        v = np.zeros(len(s[i]))
        w = np.zeros(len(s[i]))
        start_turn_index = []
        end_turn_index = []
        for j in range(0, len(s[i])-1):
            if x[i][j] == x[i][j+1] and y[i][j] == y[i][j+1]:
                # v[j] = 0
                w[j] = math.inf
            elif x[i][j] == x[i][j+1]:
                v[j] = abs(y[i][j+1] - y[i][j]) / dt
                # w[j] = 0
            elif y[i][j] == y[i][j+1]:
                v[j] = abs(x[i][j+1] - x[i][j]) / dt
                # w[j] = 0
        flag = False
        for k in range(1, len(s[i])-1):
            if w[k] == math.inf and w[k-1] != math.inf:
                start_turn_index.append(k)
                start_vector = [x[i][k] - x[i][k-10], y[i][k] - y[i][k-10]]
            elif w[k] == math.inf and w[k+1] != math.inf:
                end_turn_index.append(k)
                end_vector = [x[i][k+10] - x[i][k], y[i][k+10] - y[i][k]]
                flag = True

            if len(start_turn_index) == len(end_turn_index) and len(start_turn_index) != 0 and len(end_turn_index) != 0 and flag:
                start_vector = np.array(start_vector)
                end_vector = np.array(end_vector)
                cos_theta = np.dot(start_vector, end_vector) / (np.linalg.norm(start_vector) * np.linalg.norm(end_vector))
                w[start_turn_index[-1]:end_turn_index[-1]+1] = math.acos(cos_theta) / (dt * (end_turn_index[-1] - start_turn_index[-1]))
                flag = False

        # 创建一个字典，其中键是列名，值是对应的numpy数组
        data = {"v_mission_" + str(i): v, "w_mission_" + str(i): w}
        # 创建一个DataFrame
        df = pd.DataFrame(data)
        # 如果df_total为空，则直接将df赋值给df_total
        if df_total.empty:
            df_total = df
        else:
            # 否则，将df的列添加到df_total中
            df_total = pd.concat([df_total, df], axis=1)

        # 将DataFrame写入excel文件
    df_total.to_excel("output_total.xlsx", index=False)





def vertex_cons_2_st(vertex_cons) -> Tuple[List, List]:
    dy_obs_in = []
    dy_obs_out = []
    last = None
    for index in range(len(vertex_cons)):  # 遍历vertex_cons
        if index == 0 and vertex_cons[index] == 1:
            dy_obs_in.append((index, index))
        if last == 0 and vertex_cons[index] == 1:
            dy_obs_in.append((index, index))
        if last == 1 and vertex_cons[index] == 0:
            dy_obs_out.append((index-1, index-1))
        if index == len(vertex_cons) - 1 and vertex_cons[index] == 1:
            dy_obs_out.append((index, index))
        last = vertex_cons[index]

    # for index, k in enumerate(vertex_cons[:-1]):  # 遍历vertex_cons，但不包括最后一个元素
    #     if index == 0 and vertex_cons[index] == 1:
    #         dy_obs_in.append((index, index))
    #     if index == len(vertex_cons) - 2 and vertex_cons[index + 1] == 1:
    #         dy_obs_out.append((index, index))
    #     if k == 0 and vertex_cons[index + 1] == 1:
    #         dy_obs_in.append((index + 1, index + 1))
    #     if k == 1 and vertex_cons[index + 1] == 0:
    #         dy_obs_out.append((index, index))

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
    return values.reshape(-1, 1), times.reshape(-1, 1)


def connect_dp_paths(dp_paths_s: dict, dp_paths_t: dict, mission_index: int, corners: dict):
    dp_paths_all = np.array([])
    for j in range(len(corners[mission_index]) + 1):
        if j == 0:
            DP = np.column_stack((dp_paths_s[(mission_index, j)], dp_paths_t[(mission_index, j)]))
            dp_paths_all = DP
        else:
            DP = np.column_stack((dp_paths_s[(mission_index, j)][1:], dp_paths_t[(mission_index, j)][1:])) + dp_paths_all[-1]
            dp_paths_all = np.concatenate((dp_paths_all, DP), axis=0)
    return dp_paths_all


def connect_qp_paths(qp_paths_s: dict, qp_paths_t: dict, mission_index: int, corners: dict):
    qp_paths_all = np.array([])
    for j in range(len(corners[mission_index]) + 1):
        if j == 0:
            QP = np.column_stack((qp_paths_s[(mission_index, j)], qp_paths_t[(mission_index, j)]))
            qp_paths_all = QP
        else:
            QP = np.column_stack((qp_paths_s[(mission_index, j)], qp_paths_t[(mission_index, j)])) + qp_paths_all[-1]
            qp_paths_all = np.concatenate((qp_paths_all, QP), axis=0)
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

def semi_dynamic_obstacles_dp(dp_paths_all, path, semi_dynamic_obstacles):
    t = dp_paths_all[-1][1]
    if t not in semi_dynamic_obstacles:
        semi_dynamic_obstacles[t] = set()
        semi_dynamic_obstacles[t].add((path[-1][0], path[-1][1]))
    else:
        semi_dynamic_obstacles[t].add((path[-1][0], path[-1][1]))

    return semi_dynamic_obstacles

def check_wait_segment(path):
    if np.array_equal(path[0], path[-1]):
        return True
    else:
        return False


def main():
    starts = []
    goals = []

    #  gazebo map
    starts = [(17, 6), (3, 7), (8, 2), (4, 1), (17, 16), (3, 3), (4, 11), (1, 4), (3, 18), (6, 9), (2, 17), (16, 18), (14, 10), (18, 3), (12, 18)]
    goals = [(2, 16), (17, 12), (13, 17), (16, 14), (3, 13), (15, 8), (18, 17), (17, 2), (11, 14), (18, 13), (16, 1), (13, 1), (1, 8), (7, 17), (2, 2)]

    # small size map
    # starts = [(32, 5), (41, 14), (37, 3), (6, 6), (42, 17), (11, 2), (40, 14), (41, 1), (6, 18), (38, 11), (14, 18), (4, 1), (10, 1), (42, 16), (35, 11), (2, 15), (36, 10), (29, 14), (41, 7), (2, 17)]
    # goals = [(6, 5), (16, 17), (10, 4), (31, 17), (7, 2), (28, 17), (9, 6), (1, 6), (41, 12), (9, 4), (36, 15), (34, 12), (34, 17), (5, 9), (2, 3), (24, 18), (7, 14), (6, 17), (18, 13), (34, 16)]

    # starts = [(23, 50), (84, 33), (85, 45), (150, 58), (88, 29), (152, 22), (4, 42), (168, 4), (23, 37), (3, 75),
    #          (73, 25), (154, 43), (120, 48), (166, 67), (10, 31), (48, 80), (160, 2), (15, 36), (52, 61), (43, 21),
    #          (14, 36), (158, 1), (102, 34), (133, 82), (19, 65), (111, 42), (160, 19), (24, 31), (146, 47), (24, 35)]
    # goals = [(131, 29), (151, 76), (131, 14), (146, 9), (163, 42), (26, 65), (143, 30), (22, 17), (71, 69), (96, 51),
    #         (121, 58), (13, 11), (55, 37), (50, 70), (36, 68), (18, 33), (95, 46), (164, 65), (163, 35), (152, 5),
    #         (58, 6), (98, 74), (36, 74), (16, 38), (156, 23), (154, 14), (61, 20), (79, 78), (68, 14), (151, 74)]

    # starts = [(5, 5), (20, 15), (10, 70), (15, 35), (15, 20), (15, 10), (10, 15), (25, 80), (15, 15), (20, 55)]  # origin
    # goals = [(160, 80), (160, 60), (155, 15), (150, 75), (160, 80), (150, 20), (155, 70), (160, 70), (165, 25), (165, 10)]  # origin
    env = Env(use_benchmark=True)
    mission = Mission(starts, goals)
    planner = Planner(grid_size=1, robot_radius=0.4, static_obstacles=env.obs)
    paths = dict()
    paths_angle = dict()
    corners_index = dict()
    dynamic_obstacles = dict()
    semi_dynamic_obstacles = dict()
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

    time_of_global_planner = 0
    time_of_dp_planner = 0
    time_of_qp_planner = 0

    all_traj_length = 0
    all_traj_time = 0

    [v_max, v_min, a_max, a_min] = [3.0, 0.0, 2.0, -2.0]

    start_time_all = time.time()
    for i in tqdm(range(mission.mission_num), desc="Motion_Planner"):  # desc确认进度条的名称

        start_time_global_planner = time.time()
        # 在GlobalPlanner中规避edge_constraints，只留下vertex_constraints用于运动规划
        path, path_angle, vertex_constraints, up_bound_list, low_bound_list = planner.plan(
            start=mission.starts[i],
            goal=mission.goals[i],
            dynamic_obstacles=dynamic_obstacles,
            semi_dynamic_obstacles=semi_dynamic_obstacles,
            max_iter=1000,
            debug=False
        )
        time_of_global_planner += time.time() - start_time_global_planner
        # print("path:", path)
        # print("dynamic_obstacles", dynamic_obstacles)
        # print("up_bound_list", up_bound_list)
        # print("low_bound_list", low_bound_list)
        dp_vertex_constraints[i] = vertex_constraints
        up_dp_bound[i] = up_bound_list
        low_dp_bound[i] = low_bound_list
        # print("mission:", i, "vertex_constraints: ", vertex_constraints)

        # 计算轨迹的拐点
        corner = []
        for j in range(1, len(path_angle)):
            # if math.sqrt(math.pow(path[j - 1][0] - path[j + 1][0], 2) + math.pow(path[j - 1][1] - path[j + 1][1], 2)) != 2:
            # if path[j - 1][0] == path[j][0] == path[j + 1][0] or path[j - 1][1] == path[j][1] == path[j + 1][1]:
            #     continue
            # else:
            #     corner.append(j)

            if path_angle[j] != path_angle[j - 1]:
                corner.append(j)  # j为下一段轨迹的起始点的index
                print("j:", j, "path:[j]", path[j], "path_angle[j]:", path_angle[j], "path_angle[j-1]:", path_angle[j-1])


        corners_index[i] = corner
        paths[i] = path
        paths_angle[i] = path_angle

        s[i] = np.zeros(len(paths[i]))
        for k in range(1, len(paths[i])):
            if np.array_equal(paths[i][k], paths[i][k - 1]):
                s[i][k] = s[i][k - 1]
            else:
                s[i][k] = s[i][k - 1] + 1

        for j in range(len(corners_index[i]) + 1):
            # print("corners_index[i]:", len(corners_index[i]), "j:", j)
            if len(corners_index[i]) == 0:  # 如果没有拐点,路径为一条直线
                path_sub[(i, j)] = paths[i][0:]
                dp_vertex_constraints_sub[(i, j)] = dp_vertex_constraints[i][0:]
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][0:]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][0:]
                break
            if j == 0:
                path_sub[(i, j)] = paths[i][0:corners_index[i][j]]
                dp_vertex_constraints_sub[(i, j)] = dp_vertex_constraints[i][0:corners_index[i][j]]
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][0:corners_index[i][j]]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][0:corners_index[i][j]]
            elif j == len(corners_index[i]):
                path_sub[(i, j)] = paths[i][corners_index[i][-1]-1:]
                dp_vertex_constraints_sub[(i, j)] = np.insert(dp_vertex_constraints[i][corners_index[i][-1]:], 0, 0)
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][corners_index[i][-1]-1:]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][corners_index[i][-1]-1:]
            else:
                path_sub[(i, j)] = paths[i][corners_index[i][j-1]-1:corners_index[i][j]]
                dp_vertex_constraints_sub[(i, j)] = np.insert(dp_vertex_constraints[i][corners_index[i][j-1]:corners_index[i][j]], 0, 0)
                up_dp_bound_sub[(i, j)] = up_dp_bound[i][corners_index[i][j-1]-1:corners_index[i][j]]
                low_dp_bound_sub[(i, j)] = low_dp_bound[i][corners_index[i][j-1]-1:corners_index[i][j]]
        for index, value in enumerate(path):
            print("Index:", index, "Path:", value)
        for j in range(len(corners_index[i]) + 1):
            # print("corners_index:", corners_index[i])
            print("(i, j):", i, j, "path_sub:", path_sub[(i, j)])
            print("dp_vertex_constraints_sub:", dp_vertex_constraints_sub[(i, j)])
            if j == 0:  # 第一段
                if check_wait_segment(path_sub[(i, j)]):
                    print("wait segment")
                    dp_paths_s[(i, j)] = np.zeros((len(path_sub[(i, j)]), 1))
                    dp_paths_t[(i, j)] = np.arange(0, len(path_sub[(i, j)])).reshape(-1, 1)
                    qp_paths_s[(i, j)] = np.zeros((int(dp_paths_t[(i, j)][-1]) * 100, 1))
                    qp_paths_t[(i, j)] = np.linspace(0, dp_paths_t[(i, j)][-1], int(dp_paths_t[(i, j)][-1]) * 100).reshape(-1, 1)
                else:
                    dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])
                    start_time_dp_planner = time.time()
                    dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                                   dy_obs_in=dy_obs_in,
                                                                                   dy_obs_out=dy_obs_out,
                                                                                   v_max=v_max,
                                                                                   v_min=v_min,
                                                                                   a_max=a_max,
                                                                                   a_min=a_min,
                                                                                   dp_up_bound=up_dp_bound_sub[(i, j)],
                                                                                   dp_low_bound=low_dp_bound_sub[(i, j)])
                    time_of_dp_planner += time.time() - start_time_dp_planner

                    start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)  # t, s, v, a
                    end_state = (end_state[0], end_state[1], 0, 0)  # t, s, v, a

                    # print("end_state", end_state)

                    start_time_qp_planner = time.time()
                    osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                             start_state=start_state, end_state=end_state, dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out)
                    time_of_qp_planner += time.time() - start_time_qp_planner

                    qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])

            elif j == len(corners_index[i]):  # 最后一段
                if check_wait_segment(path_sub[(i, j)]):
                    print("wait segment")
                    dp_paths_s[(i, j)] = np.zeros((len(path_sub[(i, j)]), 1))
                    dp_paths_t[(i, j)] = np.arange(0, len(path_sub[(i, j)])).reshape(-1, 1)
                    qp_paths_s[(i, j)] = np.zeros((int(dp_paths_t[(i, j)][-1]) * 100, 1))
                    qp_paths_t[(i, j)] = np.linspace(0, dp_paths_t[(i, j)][-1], int(dp_paths_t[(i, j)][-1]) * 100).reshape(-1, 1)
                else:
                    dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])

                    start_time_dp_planner = time.time()
                    dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                                   dy_obs_in=dy_obs_in,
                                                                                   dy_obs_out=dy_obs_out,
                                                                                   v_max=v_max,
                                                                                   v_min=v_min,
                                                                                   a_max=a_max,
                                                                                   a_min=a_min,
                                                                                   dp_up_bound=up_dp_bound_sub[(i, j)],
                                                                                   dp_low_bound=low_dp_bound_sub[(i, j)])
                    time_of_dp_planner += time.time() - start_time_dp_planner

                    start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)
                    end_state = (end_state[0], end_state[1], 0, 0)

                    # print("end_state", end_state)

                    start_time_qp_planner = time.time()
                    osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                             start_state=start_state, end_state=end_state, dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out)
                    time_of_qp_planner += time.time() - start_time_qp_planner

                    qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])

            else:  # 中间段
                if check_wait_segment(path_sub[(i, j)]):
                    print("wait segment")
                    dp_paths_s[(i, j)] = np.zeros((len(path_sub[(i, j)]), 1))
                    dp_paths_t[(i, j)] = np.arange(0, len(path_sub[(i, j)])).reshape(-1, 1)
                    qp_paths_s[(i, j)] = np.zeros((int(dp_paths_t[(i, j)][-1]) * 100, 1))
                    qp_paths_t[(i, j)] = np.linspace(0, dp_paths_t[(i, j)][-1], int(dp_paths_t[(i, j)][-1]) * 100).reshape(-1, 1)
                else:
                    # print("dp_vertex_constraints_sub:", dp_vertex_constraints_sub[(i, j)])

                    dy_obs_in, dy_obs_out = vertex_cons_2_st(dp_vertex_constraints_sub[(i, j)])

                    start_time_dp_planner = time.time()
                    dp_paths_s[(i, j)], dp_paths_t[(i, j)], end_state = dp_planner(path=path_sub[(i, j)],
                                                                                   dy_obs_in=dy_obs_in,
                                                                                   dy_obs_out=dy_obs_out,
                                                                                   v_max=v_max,
                                                                                   v_min=v_min,
                                                                                   a_max=a_max,
                                                                                   a_min=a_min,
                                                                                   dp_up_bound=up_dp_bound_sub[(i, j)],
                                                                                   dp_low_bound=low_dp_bound_sub[(i, j)])
                    time_of_dp_planner += time.time() - start_time_dp_planner

                    start_state = (dp_paths_t[(i, j)][0, 0], dp_paths_s[(i, j)][0, 0], 0, 0)
                    end_state = (end_state[0], end_state[1], 0, 0)

                    # print("end_state", end_state)

                    start_time_qp_planner = time.time()
                    osqp_result = qp_planner(dp_paths_t[(i, j)], dp_paths_s[(i, j)], v_max, v_min,
                                             start_state=start_state, end_state=end_state, dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out)
                    time_of_qp_planner += time.time() - start_time_qp_planner

                    qp_paths_s[(i, j)], qp_paths_t[(i, j)] = qp_result2fun(osqp_result, dp_paths_t[(i, j)])
            # print(len(dp_paths_s[(i, j)]), len(path_sub[(i, j)]))
            print(len(dp_paths_s[(i, j)]), qp_paths_s[(i, j)].shape, qp_paths_t[(i, j)].shape)
        dp_paths_all[i] = connect_dp_paths(dp_paths_s, dp_paths_t, i, corners_index)  # (s, t)
        qp_paths_all[i] = connect_qp_paths(qp_paths_s, qp_paths_t, i, corners_index)
        print(dp_paths_all[i].shape, qp_paths_all[i].shape)

        all_traj_length += qp_paths_all[i][-1][0]
        all_traj_time += qp_paths_all[i][-1][1]

        dynamic_obstacles = dynamic_obstacles_dp(dp_paths_all[i], path, path_angle, s[i], dynamic_obstacles)
        semi_dynamic_obstacles = semi_dynamic_obstacles_dp(dp_paths_all[i], path, semi_dynamic_obstacles)

    end_time_all = time.time()
    print("Time:", end_time_all - start_time_all, "Time of Global Planner:", time_of_global_planner,
          "Time of DP Planner:", time_of_dp_planner, "Time of QP Planner:", time_of_qp_planner)
    print("Mission Num:", mission.mission_num,"All Trajectory Length:", all_traj_length, "All Trajectory Time:", all_traj_time)

    # print("dp_vertex_constraints[4]:", dp_vertex_constraints[4])
    final_x, final_y = qp_st_2_xy(path_sub, qp_paths_s, qp_paths_t, corners_index, mission.mission_num)
    s_2_v(qp_paths_all, final_x, final_y, mission.mission_num)
    # print("final_y[7]:", final_x[7])
    Plotting(mission.starts, mission.goals, mission.mission_num, env.y_range, env.x_range, env.obs, paths, paths_angle,
             corners_index, dp_paths_all, qp_paths_all, s, up_dp_bound, low_dp_bound, dp_vertex_constraints, final_x, final_y)

if __name__ == '__main__':
    main()

