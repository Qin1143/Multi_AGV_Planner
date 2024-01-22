import math
import numpy as np
from typing import List, Tuple, Dict, Callable, Set, Any


# cur_state[t, s, v, a]
def dp_planner(cur_state: Tuple[float, float, float, float], next_state: Tuple[float, float, float, float], end_state: Tuple[float, float, float, float],
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