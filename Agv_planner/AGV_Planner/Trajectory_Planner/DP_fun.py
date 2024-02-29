import math
import numpy as np
from typing import List, Tuple, Dict, Callable, Set, Any

def dp_planner(path, dy_obs_in, dy_obs_out, v_max, v_min, a_max, a_min, dp_up_bound, dp_low_bound): # dy_obs: t, s
    '''
    该函数将使用动态规划算法计算路径的速度曲线
    '''
    print("dy_obs_in:", len(dy_obs_in), dy_obs_in,"dy_obs_out:", len(dy_obs_out), dy_obs_out)
    if len(dy_obs_in) != len(dy_obs_out):
        raise ValueError("The number of dy_obs_in and dy_obs_out should be the same.")
    time_points = np.arange(len(path))
    displacement_points = []
    dis = 0
    for i in range(len(path)):
        if i == 0:
            displacement_points.append(0)
        else:
            dis += np.linalg.norm(path[i] - path[i - 1])
            displacement_points.append(dis)

    if len(dy_obs_in) == 0:
        obs_st_s_in_set = []
        obs_st_t_in_set = []
        obs_st_s_out_set = []
        obs_st_t_out_set = []
    else:
        obs_st_s_in_set = [point[1] for point in dy_obs_in]
        obs_st_t_in_set = [point[0] for point in dy_obs_in]

        obs_st_s_out_set = [point[1] for point in dy_obs_out]
        obs_st_t_out_set = [point[0] for point in dy_obs_out]

    [w_cost_ref_speed, w_cost_accel, w_cost_obs] = [2, 2, 1]
    reference_speed_unlimit = 1
    plan_start_s_dot = 0

    resolution = 1  # 目前只能是1，不可调节
    interpolated_time = np.arange(time_points[0], time_points[-1] + resolution, resolution)
    interpolated_s = np.arange(displacement_points[0], displacement_points[-1] + resolution, resolution)
    # print("interpolated_s:", interpolated_s)
    # print("interpolated_time:", interpolated_time)
    shape = (len(interpolated_s), len(interpolated_time) - 1) # 行数为s-1，列数为t-1，去除起点
    # 声明st代价矩阵，表示从起点开始到(i,j)点的最小代价为dp_st_cost(i,j)
    dp_st_cost = np.ones(shape) * math.inf
    # dp_st_s_dot，表示从起点开始到(i,j)点的最优路径的末速度为dp_st_s_dot(i,j)
    dp_st_s_dot = np.zeros(shape)
    # dp_st_s_dot2，表示从起点开始到(i,j)点的最优路径的末加速度为dp_st_s_dot2(i,j)
    dp_st_s_dot2 = np.zeros(shape)
    # 需要一个矩阵保持最优路径的前一个节点方便回溯
    # dp_st_node(i, j)表示位置为(i, j)的节点中，最优的上一层节点的行号为dp_st_node(i, j)
    dp_st_node = np.zeros(shape)
    # print("interpolated_s:", interpolated_s)
    # print("dp_up_bound:", dp_up_bound)
    # print("dp_low_bound:", dp_low_bound)
    # 计算从dp起点到第一列的cost
    for i in range(len(interpolated_s)):

        if interpolated_s[i] > interpolated_s[1] + dp_up_bound[1] or interpolated_s[i] <  interpolated_s[1] + dp_low_bound[1]:
            continue

        dp_st_cost[i, 0], cur_s_dot, cur_s_dot2 = CalcDpCost(0,0, i,0, obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set,
                                      w_cost_ref_speed, reference_speed_unlimit, w_cost_accel, w_cost_obs, plan_start_s_dot, interpolated_s,
                                      interpolated_time, dp_st_s_dot, v_max, v_min, a_max, a_min)
        dp_st_s_dot[i, 0] = cur_s_dot
        dp_st_s_dot2[i, 0] = cur_s_dot2

    # 动态规划主程序
    for j in range(len(interpolated_time) - 1):
        # j为列循环
        for i in range(len(interpolated_s)):
            # i为行循环
            # 当前为i行j列
            cur_row = i
            cur_col = j
            if interpolated_s[i] > interpolated_s[cur_col + 1] + dp_up_bound[cur_col + 1] or interpolated_s[i] < interpolated_s[cur_col + 1] + dp_low_bound[cur_col + 1]:
                continue

            # 遍历前一列
            for k in range(len(interpolated_s)):
                pre_row = k
                pre_col = j - 1
                if interpolated_s[i] > interpolated_s[cur_col + 1] + dp_up_bound[cur_col + 1] or interpolated_s[i] < interpolated_s[cur_col + 1] + dp_low_bound[cur_col + 1]:
                    continue

                # 计算边的代价，其中起点为(k, j-1)，终点为(i, j)
                cost_temp, cur_s_dot, cur_s_dot2 = CalcDpCost(pre_row, pre_col, cur_row, cur_col, obs_st_s_in_set, obs_st_s_out_set,
                                       obs_st_t_in_set, obs_st_t_out_set, w_cost_ref_speed, reference_speed_unlimit, w_cost_accel, w_cost_obs,
                                       plan_start_s_dot, interpolated_s, interpolated_time, dp_st_s_dot, v_max, v_min, a_max, a_min)
                if cost_temp + dp_st_cost[pre_row, pre_col] < dp_st_cost[cur_row, cur_col]:
                    dp_st_cost[cur_row, cur_col] = cost_temp + dp_st_cost[pre_row, pre_col]
                    # 计算最优的s_dot
                    (s_start, t_start) = (interpolated_s[pre_row], interpolated_time[pre_col + 1])
                    (s_end, t_end) = (interpolated_s[cur_row], interpolated_time[cur_col + 1])
                    # dp_st_s_dot[cur_row, cur_col] = (s_end - s_start) / (t_end - t_start)
                    dp_st_s_dot[cur_row, cur_col] = cur_s_dot
                    dp_st_s_dot2[cur_row, cur_col] = cur_s_dot2
                    # 将最短路径的前一个节点的行号记录下来
                    dp_st_node[cur_row, cur_col] = pre_row
    # 输出初始化
    dp_speed_s = np.zeros((len(interpolated_time), 1))
    dp_speed_t = np.zeros((len(interpolated_time), 1))
    dp_speed_s[0] = interpolated_s[0]
    dp_speed_t[0] = interpolated_time[0]
    # 找到dp_node_cost 上边界和右边界代价最小的节点
    min_cost = math.inf
    min_row = int
    min_col = int
    # 遍历右边界
    # for i in range(len(interpolated_s)):
    #     if dp_st_cost[i, -1] < min_cost:
    #         min_cost = dp_st_cost[i, -1]
    #         min_row = i
    #         min_col = len(interpolated_time) - 2
    # 遍历上边界
    for j in range(len(interpolated_time) - 1):
        if dp_st_cost[-1, j] <= min_cost:
            min_cost = dp_st_cost[-1, j]
            min_row = len(interpolated_s) - 1
            min_col = j
    if min_col +2 != len(dp_speed_s):
        cut_length = len(dp_speed_s) - (min_col + 2)
        dp_speed_s = dp_speed_s[:-cut_length]
        dp_speed_t = dp_speed_t[:-cut_length]
    dp_speed_s[min_col+1] = interpolated_s[min_row]
    dp_speed_t[min_col+1] = interpolated_time[min_col+1]
    end_t = interpolated_time[min_col+1]
    ens_s = interpolated_s[min_row]
    end_v = dp_st_s_dot[min_row, min_col]
    end_a = dp_st_s_dot2[min_row, min_col]
    end_state = (int(end_t), int(ens_s), end_v, end_a)

    # 反向回溯

    while min_col != 0:
        # print(min_row, min_col)
        pre_row = int(dp_st_node[min_row, min_col])
        pre_col = min_col - 1
        dp_speed_s[pre_col+1] = interpolated_s[pre_row]
        dp_speed_t[pre_col+1] = interpolated_time[pre_col+1]
        min_row = pre_row
        min_col = pre_col
    print("DP Finished!")
    # print("dp_st_node \n", dp_st_node)
    # print("dp_st_cost \n", dp_st_cost)
    # print("dp_speed_s \n", dp_speed_s)
    return dp_speed_s, dp_speed_t, end_state

def CalcDpCost(
    row_start, col_start, row_end, col_end, obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set,
        w_cost_ref_speed, reference_speed, w_cost_accel, w_cost_obs, plan_start_s_dot, s_list, t_list, dp_st_s_dot, v_max, v_min, a_max, a_min
):
    '''
    该函数将计算链接两个节点之间边的代价
    :param row_start: 边的起点的行列号
    :param col_start: 边的起点的行列号
    :param row_end: 边的终点行列号
    :param col_end: 边的终点行列号
    :param obs_st_s_in_set: 障碍物st信息
    :param obs_st_s_out_set: 障碍物st信息
    :param obs_st_t_in_set: 障碍物st信息
    :param obs_st_t_out_set: 障碍物st信息
    :param w_cost_ref_speed: 参考速度代价权重
    :param reference_speed: 参考速度
    :param w_cost_accel: 加速度代价权重
    :param w_cost_obs: 障碍物代价权重
    :param plan_start_s_dot: 起点速度
    :param s_list: 采样点
    :param t_list: 采样点
    :param dp_st_s_dot: 用于计算加速度
    :param v_max: 最大速度
    :param v_min: 最小速度
    :param a_max: 最大加速度
    :param a_min: 最小加速度
    :return:
    '''
    # 首先计算终点的st坐标
    [s_end, t_end] = [s_list[row_end], t_list[col_end+1]]
    # 起点的行序列号为0
    if row_start == 0:
        s_start = 0
        t_start = 0
        s_dot_start = plan_start_s_dot
    else:
        # 边的起点不是DP的起点
        [s_start, t_start] = [s_list[row_start], t_list[col_start+1]]
        s_dot_start = dp_st_s_dot[row_start, col_start]
    cur_s_dot = (s_end - s_start) / (t_end - t_start)
    cur_s_dot2 = (cur_s_dot - s_dot_start) / (t_end - t_start)
    # 计算推荐速度代价
    if cur_s_dot <= v_max and cur_s_dot >= v_min:
        cost_ref_speed = w_cost_ref_speed * (cur_s_dot - reference_speed) ** 2
    else:
        cost_ref_speed = math.inf
    # 计算加速度代价
    if cur_s_dot2 <= a_max and cur_s_dot2 >= a_min:
        cost_accel = w_cost_accel * cur_s_dot2 ** 2
    else:
        cost_accel = math.inf
    # 计算障碍物代价

    cost_obs = CalcObsCost(s_start, t_start, s_end, t_end, obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set, w_cost_obs)
    cost = cost_ref_speed + cost_accel + cost_obs
    return cost, cur_s_dot, cur_s_dot2

def CalcObsCost(
    s_start,t_start,s_end,t_end,obs_st_s_in_set,obs_st_s_out_set,obs_st_t_in_set,obs_st_t_out_set,w_cost_obs
):
    '''
    该函数将计算当前点与多个障碍物的代价
    :param s_start: 边的起点
    :param t_start: 边的起点
    :param s_end: 边的终点
    :param t_end: 边的终点
    :param obs_st_s_in_set: 障碍物信息
    :param obs_st_s_out_set: 障碍物信息
    :param obs_st_t_in_set: 障碍物信息
    :param obs_st_t_out_set: 障碍物信息
    :param w_cost_obs: 障碍物代价权重
    :return: 边的障碍物代价
    '''
    obs_cost = 0
    if len(obs_st_s_in_set) == 0:
        return obs_cost
    # 采样点数量
    n = 5
    # 采样点时间间隔
    dt = (t_end - t_start) / (n - 1)
    # 边的斜率
    k = (s_end - s_start) / (t_end - t_start)
    for i in range(n):
        # 计算采样点
        s = s_start + k * dt * i
        t = t_start + dt * i
        # 遍历所有障碍物
        for j in range(len(obs_st_s_in_set)):
            if obs_st_s_in_set[j] == None:
                continue
            # 计算点到线段的最段距离
            cur_point = np.array([t, s])
            obs_point_1 = np.array([obs_st_s_in_set[j], obs_st_t_in_set[j]])
            obs_point_2 = np.array([obs_st_s_out_set[j], obs_st_t_out_set[j]])

            V1 = obs_point_1 - cur_point
            V2 = obs_point_2 - obs_point_1
            V3 = obs_point_2 - cur_point

            epsilon = 1e-7
            d1 = np.linalg.norm(V1)
            d2 = np.linalg.norm(V3)
            d3 = np.linalg.norm(np.cross(V1, V2)) / (np.linalg.norm(V2) + epsilon)

            if np.dot(V1, V2) * np.dot(V1, V3) >= 0:
                min_dis = min(d1, d2)
            else:
                min_dis = d3
            obs_cost = obs_cost + CalcCollisionCost(w_cost_obs, min_dis)
    return obs_cost

def CalcCollisionCost(w_cost_obs, min_dis):
    obs_l_bound = 0.2
    obs_u_bound = 1.0
    if abs(min_dis) < obs_l_bound:
        collision_cost = w_cost_obs
    elif abs(min_dis) > obs_l_bound and abs(min_dis) < obs_u_bound:
    # min_dis = 0.5 collision_cost = w_cost_obs ** 1;
    # min_dis = 1.5 collision_cost = w_cost_obs ** 0 = 1
        collision_cost = w_cost_obs ** ((obs_l_bound - min_dis) + obs_u_bound - obs_l_bound)
    else:
        collision_cost = 0
    return collision_cost