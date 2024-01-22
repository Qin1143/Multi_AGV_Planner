import math
import numpy as np
from typing import List, Tuple, Dict, Callable, Set, Any

def dp_planner(path, path_start_time, path_end_time, dy_obs, v_max, v_min, a_max, a_min): # dy_obs: t, x, y
    obs_state = []
    time_points = np.arange(len(path))
    displacement_points = []
    dis = 0
    for i in range(len(path)):
        if i == 0:
            displacement_points.append(0)
        else:
            dis += np.linalg.norm(path[i] - path[i - 1])
            displacement_points.append(dis)

    resolution = 1
    interpolated_time = np.arange(time_points[0], time_points[-1] + resolution, resolution)
    interpolated_s = np.arange(displacement_points[0], displacement_points[-1] + resolution, resolution)
    shape = (len(interpolated_s), len(interpolated_time))# 行数为s，列数为t
    J_cost2go = np.zeros((shape))
    state_matrix = np.empty((shape), dtype=object)
    parent = dict()
    dp_points = []
    dp_points_t = []
    dp_points_s = []
    # state: (t, s, v, a)
    start_state = (interpolated_time[0], interpolated_s[0], 0, 0)
    end_state = (interpolated_time[-1], interpolated_s[-1], 0, 0)
    next_state = ()
    i =  0
    best_start_start = None
    for t in interpolated_time:
        j = 0
        if t == interpolated_time[0]: # 第一列
            state_matrix[j, i] = start_state
            parent[start_state] = start_state
            i += 1
        elif t == interpolated_time[1]: # 第二列
            for s in interpolated_s:
                next_state = (t, s, 0, 0)
                J_cost, next_state = dp_one_level(cur_state=start_state, next_state=next_state, end_state=end_state, obs_state=obs_state, ref_velocity=1, v_max=v_max, v_min=v_min, a_max=a_max, a_min=a_min)
                J_cost2go[j, i] = J_cost
                state_matrix[j, i] = next_state
                parent[next_state] = start_state
                j += 1
            i += 1
        else:
            for s in interpolated_s:
                J_cost_min = math.inf
                k = 0
                for cur_state in state_matrix[:, i-1]:
                    if cur_state == None:
                        continue
                    next_state = (t, s, 0, 0)
                    J_cost, next_state = dp_one_level(cur_state=cur_state, next_state=next_state, end_state=end_state, obs_state=obs_state, ref_velocity=1, v_max=v_max, v_min=v_min, a_max=a_max, a_min=a_min)
                    J_cost = J_cost + J_cost2go[k, i-1]
                    if J_cost < J_cost_min:
                        J_cost_min = J_cost
                        state_matrix[j, i] = next_state
                        best_start_start = cur_state
                    k += 1
                parent[state_matrix[j, i]] = best_start_start
                J_cost2go[j, i] = J_cost_min
                j += 1
            i += 1

            # plt.plot(best_state[0], best_state[1], marker='o', color='red')
    print("Finish DP")
    min_index_last_column = np.argmin(J_cost2go[:, -1])
    best_state = state_matrix[min_index_last_column, -1]
    end_state = best_state
    while True:
        # dp_points_t.append(best_state[0])
        # dp_points_s.append(best_state[1])
        dp_points.append((best_state[0], best_state[1]))
        best_state = parent[best_state]
        if best_state == start_state:
            # dp_points_t.append(best_state[0])
            # dp_points_s.append(best_state[1])
            dp_points.append((best_state[0], best_state[1]))
            break
    # dp_points_t.reverse()
    # dp_points_s.reverse()
    dp_points.reverse()
    return dp_points, start_state, end_state, obs_state


# cur_state[t, s, v, a]
def dp_one_level(cur_state: Tuple[float, float, float, float], next_state: Tuple[float, float, float, float], end_state: Tuple[float, float, float, float],
                 obs_state: List[Tuple[float, float]], ref_velocity: float, v_max: float, v_min: float, a_max: float, a_min: float) -> Tuple[float, Tuple[float, float, float, float]]:
    dt = next_state[0] - cur_state[0]
    '''
    w1: cost_obs
    w2: cost_acc
    w3: cost_jerk
    w4: cost_ref_speed
    w5: cost_end
    '''
    [w1, w2, w3, w4, w5] = [1, 10, 1, 300, 60]

    ds = (next_state[1] - cur_state[1]) / dt
    dds = (ds - cur_state[2]) / dt
    ddds = (dds - cur_state[3]) / dt

    if ds > v_max or ds < v_min:
        J_cost = math.inf
        return J_cost, next_state

    if dds > a_max or dds < a_min:
        J_cost = math.inf
        return J_cost, next_state

    import numpy as np

    if len(obs_state) == 0:
        cost_obs = 0
    else:
        # Generate 4 equally spaced points between cur_state and next_state
        state_points = np.linspace(cur_state, next_state, num=6)[1:-1]
        costs = 0
        for point in state_points:
            cost = cost_obs_fun(point, obs_state)    # Calculate cost_obs_fun for each point
            costs += cost
        cost_obs = w1 * costs

    cost_acc = w2 * dds ** 2
    cost_jerk = w3 * ddds ** 2
    cost_ref_speed = w4 * (ds - ref_velocity) ** 2
    cost_end = w5 * ((end_state[0] - next_state[0]) ** 2 + (end_state[1] - next_state[1]) ** 2)

    J_cost = round(cost_obs + cost_acc + cost_jerk + cost_ref_speed + cost_end, 2)
    next_state = (next_state[0], next_state[1], ds, dds)

    return J_cost, next_state
def cost_obs_fun(cur_state: Tuple[float, float, float, float], obs_state: List[Tuple[float, float]]) -> float:

    state_point = np.array([cur_state[0], cur_state[1]])
    obs_point_1 = np.array([obs_state[0][0], obs_state[0][1]])
    obs_point_2 = np.array([obs_state[1][0], obs_state[1][1]])

    V1 = obs_point_1 - state_point
    V2 = obs_point_2 - obs_point_1
    V3 = obs_point_2 - state_point

    d1 = np.linalg.norm(V1)
    d2 = np.linalg.norm(V3)
    d3 = np.linalg.norm(np.cross(V1, V2)) / np.linalg.norm(V2)

    if np.dot(V1,V2) * np.dot(V1,V3) >= 0:
        min_d = min(d1, d2)
    else:
        min_d = d3

    if abs(min_d) <= 0.5:
        cost_obs = 10000
    elif min_d > 8:
        cost_obs = 0
    else:
        cost_obs = 1/min_d
    return cost_obs