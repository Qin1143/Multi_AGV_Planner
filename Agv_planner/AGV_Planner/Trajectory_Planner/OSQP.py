import osqp
import math
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

def qp_planner_osqp(dp_points_t, dp_points_s, v_max, v_min, start_state, end_state, dy_obs_in = [], dy_obs_out = []):
    # dy_obs_in为st坐标 其他的st图内的坐标为ts
    # 目前QP可以处理多障碍物，但是只能处理顺次出现的障碍物，不能处理st中有时间交叉的障碍物
    # 权重参数
    # w1: cost_acc
    # w2: cost_jerk
    # w3: cost_ref_s
    global is_collision
    [w1, w2, w3] = [1, 1, 3]
    start_collision_index = dict()
    end_collision_index = dict()
    speedup = dict()

    # DP和QP的离散时间和阶段数量
    deltT_DP = 1
    deltT_QP = 0.1
    N_DP = len(dp_points_t) - 1
    N_QP = int(deltT_DP / deltT_QP)

    no_collision = False
    if len(dy_obs_in) == 0:
        no_collision = True
    else:
        for i in range(len(dy_obs_in)):
            for j in range(len(dp_points_t)):
                if dp_points_t[j] <= dy_obs_in[i][1] < dp_points_t[j + 1] and i not in start_collision_index.keys():  # 第i个障碍物段的起始时间
                    start_collision_index[i] = j
                    if dp_points_s[j] > dy_obs_in[i][0]:
                        speedup[i] = True
                    else:
                        speedup[i] = False

                if dp_points_t[j-1] < dy_obs_out[i][1] <= dp_points_t[j] and i not in end_collision_index.keys():  # 第i个障碍物段的终止时间
                    end_collision_index[i] = j

                if j == len(dp_points_t) - 1 and i not in end_collision_index.keys():  # 如果障碍物段的终止时间在最后一个DP段的时间之后
                    end_collision_index[i] = j

                if i in start_collision_index.keys() and i in end_collision_index.keys():
                    break



        # for i in range(N_DP):
        #     for j in range(len(dy_obs_in)):  # j代表第j个障碍物段
        #         if dp_points_t[i] < dy_obs_in[j][1] <= dp_points_t[i + 1]:
        #             start_collision_index[j] = i
        #             if dp_points_s[i] > dy_obs_in[j][0]:
        #                 speedup[j] = True
        #             else:
        #                 speedup[j] = False
        #         if dp_points_t[i - 1] <= dy_obs_out[j][1] < dp_points_t[i]:
        #             end_collision_index[j] = i

    # print("dp_points_t", dp_points_t)
    # print("dy_obs_in", dy_obs_in)
    # print("dy_obs_out", dy_obs_out)
    # print("start_collision_index", start_collision_index)
    # print("end_collision_index", end_collision_index)



    # 约束
    v_init = start_state[2]
    a_init = start_state[3]

    # 目标函数
    P = np.zeros((6 * N_DP, 6 * N_DP))
    q = np.zeros((6 * N_DP, 1))
    t = deltT_DP  # 每段多项式的终止时间
    for i in range(N_DP):
        # 加速度指标
        P1 = np.array([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 4 * t, 6 * t ** 2, 8 * t ** 3, 10 * t ** 4],
                       [0, 0, 6 * t ** 2, 12 * t ** 3, 18 * t ** 4, 24 * t ** 5],
                       [0, 0, 8 * t ** 3, 18 * t ** 4, (144/5) * t ** 5, 40 * t ** 6],
                       [0, 0, 10 * t ** 4, 24 * t ** 5, 40 * t ** 6, (400/7) * t ** 6]])
        P1 = 2 * P1

        # 加加速度指标
        P2 = np.array([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 36 * t, 72 * t ** 2, 120 * t ** 3],
                       [0, 0, 0, 72 * t ** 2, 192 * t ** 3, 360 * t ** 4],
                       [0, 0, 0, 120 * t ** 3, 360 * t ** 4, 720 * t ** 5]])
        P2 = 2 * P2

        # 与动态规划的偏差指标
        P3 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5],
                        [t, t ** 2, t ** 3, t ** 4, t ** 5, t ** 6],
                        [t ** 2, t ** 3, t ** 4, t ** 5, t ** 6, t ** 7],
                        [t ** 3, t ** 4, t ** 5, t ** 6, t ** 7, t ** 8],
                        [t ** 4, t ** 5, t ** 6, t ** 7, t ** 8, t ** 9],
                        [t ** 5, t ** 6, t ** 7, t ** 8, t ** 9, t ** 10]])
        P3 = 2 * P3
        P_sub = w1 * P1 + w2 * P2 + w3 * P3
        idx1 = i * 6
        idx2 = (i + 1) * 6
        P[idx1:idx2, idx1:idx2] = P_sub
        P = sparse.csc_matrix(P)

        refS = dp_points_s[i + 1]
        q_sub = -2 * refS * w3 * np.array([[1], [t], [t**2], [t**3], [t**4], [t**5]])
        q[idx1:idx2] = q_sub

    # 等式约束
    Aeq1 = np.zeros((4 * (N_DP - 1), 6 * N_DP))
    Beq1 = np.zeros((4 * (N_DP - 1), 1))
    for i in range(N_DP - 1):
        t = deltT_DP
        # 位置约束
        eq1 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5, -1, 0, 0, 0, 0, 0]])
        # 速度约束
        eq2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4, 0, -1, 0, 0, 0, 0]])
        # 加速度约束
        eq3 = np.array([[0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3, 0, 0, -2, 0, 0, 0]])
        # 加加速度约束
        eq4 = np.array([[0, 0, 0, 6, 24 * t, 60 * t ** 2, 0, 0, 0, -6, 0, 0]])

        beq = np.array([[0], [0], [0], [0]])
        startRow = i*4
        endRow = i*4 + 4
        startCol = i*6
        endCol = i*6 + 12
        Aeq1[startRow:endRow, startCol:endCol] = np.concatenate((eq1, eq2, eq3, eq4), axis=0)
        Beq1[startRow:endRow] = beq

    # ST曲线的起点和终点约束
    Aeq2 = np.zeros((3, 6 * N_DP))
    Beq2 = np.zeros((3, 1))
    Aeq3 = np.zeros((3, 6 * N_DP))
    Beq3 = np.zeros((3, 1))
    t = 0
    # 位置约束
    eq1 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])
    # 速度约束
    eq2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])
    # 加速度约束
    eq3 = np.array([[0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3,]])

    beq = np.array([ [start_state[1]], [v_init], [a_init] ])
    Aeq2[0:3, 0:6] = np.concatenate((eq1, eq2, eq3), axis=0)
    Beq2[0:3] = beq

    t = 1
    # 位置约束
    eq1 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])
    # 速度约束
    eq2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])
    # 加速度约束
    eq3 = np.array([[0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3,]])

    beq_end = np.array([ [end_state[1]], [end_state[2]], [end_state[3]] ])
    Aeq3[0:3, -6:] = np.concatenate((eq1, eq2, eq3), axis=0)
    Beq3[0:3] = beq_end

    # 不等式约束
    # 凸空间约束
    A1 = np.zeros((1 * N_DP * N_QP, 6 * N_DP))
    bl_1 = np.zeros((1 * N_DP * N_QP, 1))
    bu_1 = np.zeros((1 * N_DP * N_QP, 1))
    for i in range(N_DP):
        for j in range(N_QP):
            t = j * deltT_QP
            if no_collision:
                # bu_1[i*N_QP + j, 0] = s_ub[i] + (s_ub[i+1]-s_ub[i]) * t / deltT_DP
                # bl_1[i*N_QP + j, 0] = s_lb[i] + (s_lb[i+1]-s_lb[i]) * t / deltT_DP
                bu_1[i*N_QP + j, 0] = end_state[1]
                bl_1[i*N_QP + j, 0] = 0
            else:
                is_collision = False
                for collision_index in start_collision_index.keys():
                    if start_collision_index[collision_index] <= i <= end_collision_index[collision_index]:
                        is_collision = True
                        break

                if is_collision == False:
                    bu_1[i*N_QP + j, 0] = end_state[1]
                    bl_1[i*N_QP + j, 0] = 0
                    time = 0
                else:
                    time += deltT_QP
                    if speedup[collision_index]:
                        bu_1[i*N_QP + j, 0] = end_state[1]
                        bl_1[i*N_QP + j, 0] = dy_obs_in[collision_index][0] + \
                        (dy_obs_out[collision_index][0] - dy_obs_in[collision_index][0]) * time / (dy_obs_out[collision_index][1] - dy_obs_in[collision_index][1])
                    else:
                        bu_1[i*N_QP + j, 0] = dy_obs_in[0][0] + \
                        (dy_obs_out[collision_index][0] - dy_obs_in[collision_index][0]) * time / (dy_obs_out[collision_index][1] - dy_obs_in[collision_index][1])
                        bl_1[i*N_QP + j, 0] = 0
            A1[i*N_QP + j, i*6 : i * 6 + 6] = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])
    # plot_bounds(bl_1, bu_1)

    # 速度约束
    A2 = np.zeros((1 * N_DP * N_QP, 6 * N_DP))
    bl_2 = np.zeros((1 * N_DP * N_QP, 1))
    bu_2 = np.zeros((1 * N_DP * N_QP, 1))
    for i in range(N_DP):
        for j in range(N_QP):
            t = j * deltT_QP
            bu_2[i*N_QP + j, 0] = v_max
            bl_2[i*N_QP + j, 0] = v_min
            A2[i*N_QP + j, i*6 : i * 6 + 6] = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])

    A = sparse.csc_matrix(np.concatenate((Aeq1, Aeq2, Aeq3, A1, A2), axis=0))
    bl = np.concatenate((Beq1, Beq2, Beq3, bl_1, bl_2), axis=0)
    bu = np.concatenate((Beq1, Beq2, Beq3, bu_1, bu_2), axis=0)

    # 检查下界是否大于上界
    invalid_bounds = np.where(bl > bu)[0]
    if invalid_bounds.size > 0:
        print(f"下界大于上界的索引：{invalid_bounds}")
        print(f"对应的下界值：{bl[invalid_bounds]}")
        print(f"对应的上界值：{bu[invalid_bounds]}")

    # print(bl.shape, bu.shape, A.shape)
    # print(Beq1.shape, Beq2.shape, Beq3.shape, bl_1.shape, bl_2.shape)
    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace and change alpha parameter
    prob.setup(P, q, A, bl, bu)#, verbose=False)

    # Solve problem
    result = prob.solve()
    # print(result)
    print("QP Finished!")
    return result.x

def plot_bounds(bl_1, bu_1):
    plt.figure(figsize=(10, 5))
    plt.plot(bl_1, label='Lower Bound')
    plt.plot(bu_1, label='Upper Bound')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Lower and Upper Bounds')
    plt.legend()
    plt.show()