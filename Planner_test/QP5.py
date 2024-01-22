from cvxopt import matrix, solvers
import numpy as np


def qp_planner(dp_points_t, dp_points_s, s_lb, s_ub, v_max, v_min, start_state, end_state):
    # 权重参数
    # w1: cost_acc
    # w2: cost_jerk
    # w3: cost_ref_s
    [w1, w2, w3] = [1, 1, 1]

    # DP和QP的离散时间和阶段数量
    deltT_DP = 1
    deltT_QP = 0.1
    N_DP = len(dp_points_t) - 1
    N_QP = int(deltT_DP / deltT_QP)

    # 约束
    v_init = start_state[2]
    a_init = start_state[3]

    # 目标函数
    H = matrix(np.zeros((6 * N_DP, 6 * N_DP)))
    f = matrix(np.zeros((6 * N_DP, 1)))
    t = deltT_DP  # 每段多项式的终止时间
    for i in range(N_DP):
        # 加速度指标
        H1 = np.array([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 4 * t, 6 * t ** 2, 8 * t ** 3, 10 * t ** 4],
                       [0, 0, 6 * t ** 2, 12 * t ** 3, 18 * t ** 4, 24 * t ** 5],
                       [0, 0, 8 * t ** 3, 18 * t ** 4, (144/5) * t ** 5, 40 * t ** 6],
                       [0, 0, 10 * t ** 4, 24 * t ** 5, 40 * t ** 6, (400/7) * t ** 6]])
        H1 = 2 * H1

        # 加加速度指标
        H2 = np.array([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 36 * t, 72 * t ** 2, 120 * t ** 3],
                       [0, 0, 0, 72 * t ** 2, 192 * t ** 3, 360 * t ** 4],
                       [0, 0, 0, 120 * t ** 3, 360 * t ** 4, 720 * t ** 5]])
        H2 = 2 * H2

        # 与动态规划的偏差指标
        H3 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5],
                        [t, t ** 2, t ** 3, t ** 4, t ** 5, t ** 6],
                        [t ** 2, t ** 3, t ** 4, t ** 5, t ** 6, t ** 7],
                        [t ** 3, t ** 4, t ** 5, t ** 6, t ** 7, t ** 8],
                        [t ** 4, t ** 5, t ** 6, t ** 7, t ** 8, t ** 9],
                        [t ** 5, t ** 6, t ** 7, t ** 8, t ** 9, t ** 10]])
        H3 = 2 * H3
        H_sub = w1 * H1 + w2 * H2 + w3 * H3
        idx1 = i * 6
        idx2 = (i + 1) * 6
        H[idx1:idx2, idx1:idx2] = H_sub

        refS = dp_points_s[i + 1]
        f_sub = -2 * refS * w3 * np.array([[1], [t], [t**2], [t**3], [t**4], [t**5]])
        f[idx1:idx2] = f_sub

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
    Aeq2 = matrix(np.zeros((3, 6 * N_DP)))
    Beq2 = matrix(np.zeros((3, 1)))
    Aeq3 = matrix(np.zeros((3, 6 * N_DP)))
    Beq3 = matrix(np.zeros((3, 1)))
    t = 0
    # 位置约束
    eq1 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])
    # 速度约束
    eq2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])
    # 加速度约束
    eq3 = np.array([[0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3,]])

    beq = np.array([[start_state[1]], [v_init], [a_init]])
    Aeq2[0:3, 0:6] = np.concatenate((eq1, eq2, eq3), axis=0)
    Beq2[0:3] = beq

    t = 1
    # 位置约束
    eq1 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])
    # 速度约束
    eq2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])
    # 加速度约束
    eq3 = np.array([[0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3,]])

    beq_end = np.array([end_state[1], end_state[2], end_state[3]])
    Aeq3[0:3, -6:] = np.concatenate((eq1, eq2, eq3), axis=0)
    Beq3[0:3] = beq_end

    Aeq = matrix(np.concatenate((Aeq1, Aeq2, Aeq3), axis=0))
    Beq = matrix(np.concatenate((Beq1, Beq2, Beq3), axis=0))

    # 不等式约束
    # 凸空间约束
    A1 = np.zeros((2 * N_DP * N_QP, 6 * N_DP))
    b1 = np.zeros((2 * N_DP * N_QP, 1))
    for i in range(N_DP):
        for j in range(N_QP):
            t = j * deltT_QP
            ub = s_ub[i] + (s_ub[i+1]-s_ub[i]) * t / deltT_DP
            lb = s_lb[i] + (s_lb[i+1]-s_lb[i]) * t / deltT_DP

            a1 = np.array([[-1, -t, -t ** 2, -t ** 3, -t ** 4, -t ** 5]])
            a2 = np.array([[1, t, t ** 2, t ** 3, t ** 4, t ** 5]])

            startRow = i * 2 * N_QP + j * 2
            endRow = i * 2 * N_QP + j * 2 + 2
            startCol = i * 6
            endCol = i * 6 + 6

            A1[startRow:endRow, startCol:endCol] = np.concatenate((a1, a2), axis=0)
            b1[startRow:endRow] = np.array([[-lb], [ub]])

    # 速度约束
    A2 = np.zeros((2 * N_DP * N_QP, 6 * N_DP))
    b2 = np.zeros((2 * N_DP * N_QP, 1))
    for i in range(N_DP):
        for j in range(N_QP):
            t = j * deltT_QP
            ub = v_max
            lb = v_min

            a1 = np.array([[0, -1, -2 * t, -3 * t ** 2, -4 * t ** 3, -5 * t ** 4]])
            a2 = np.array([[0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]])

            startRow = i * 2 * N_QP + j * 2
            endRow = i * 2 * N_QP + j * 2 + 2
            startCol = i * 6
            endCol = i * 6 + 6

            A2[startRow:endRow, startCol:endCol] = np.concatenate((a1, a2), axis=0)
            b2[startRow:endRow] = np.array([[-lb], [ub]])

    A = matrix(np.concatenate((A1, A2), axis=0))
    b = matrix(np.concatenate((b1, b2), axis=0))

    H_np = np.array(H)
    # 计算特征值
    eigvals = np.linalg.eigvals(H_np)

    if np.all(eigvals > 0):
        print("H矩阵是正定的")
    elif np.all(eigvals >= 0):
        print("H矩阵是半正定的")
    elif np.all(eigvals < 0):
        print("H矩阵是负定的")
    else:
        print("H矩阵既不是正定的，也不是半正定的，也不是负定的")

    # 将cvxopt矩阵转换为numpy矩阵
    f_np = np.array(f.T)
    A_np = np.array(A)
    Aeq_np = np.array(Aeq)
    # 将f, A, Aeq三个矩阵整合在一起
    combined_matrix = np.concatenate((f_np, A_np, Aeq_np), axis=0)
    # 计算矩阵的秩
    rank = np.linalg.matrix_rank(combined_matrix.T)
    # 检查矩阵的秩是否等于其列数
    if rank == combined_matrix.T.shape[1]:
        print("整合后的矩阵的列是满秩的")
    else:
        print("整合后的矩阵的列不是满秩的")

    Aeq_np = np.array(Aeq)
    # 计算矩阵的秩
    rank = np.linalg.matrix_rank(Aeq_np)
    # 检查矩阵的秩是否等于其行数或列数
    if rank == Aeq_np.shape[0]:
        print("Aeq矩阵的行是满秩的")
    else:
        print("Aeq矩阵的行不是满秩的")

    # 求解QP问题
    options = {'show_progress': True, 'maxiters': 100, 'abstol': 1e-9, 'reltol': 1e-9, 'refinement': 20}
    sol = solvers.qp(H, f, A, b, Aeq, Beq, options=options)    # 返回一个字典 'x'：一个cvxopt矩阵，包含了问题的解。这个矩阵的大小和你的问题的变量数量相同。
    print(sol)
    return sol['x']