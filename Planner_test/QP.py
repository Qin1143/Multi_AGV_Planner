from cvxopt import matrix, solvers
import numpy as np

# start_state[t, s, v, a]
def qp_planner(dp_points_t, dp_points_s, s_lb, s_ub, s_d_lb, s_d_ub, ref_speed, a_max, a_min, start_state, end_state):
    # np.vstack 垂直（按照行顺序）的把数组给堆叠起来
    # np.hstack 水平（按照列顺序）的把数组给堆叠起来

    qp_size = len(dp_points_t)
    [w1, w2, w3] = [0.5, 2, 0.5]
    A_ref = matrix(np.zeros((3*qp_size, 3*qp_size)))
    A_acc = matrix(np.zeros((3*qp_size, 3*qp_size)))
    A_jerk = matrix(np.zeros((3*qp_size, qp_size - 1)))
    A_jerk_sub = np.array([0, 0, 1, 0, 0, -1])
    for i in range(qp_size):
        A_ref[3*i+1, 3*i+1] = 1
        A_acc[3*i+2, 3*i+2] = 1
    for i in range(qp_size - 1):
        A_jerk[3*i : 3*i+5 +1, i] = A_jerk_sub
    Q = w1* A_ref* A_ref.T + w2 * A_acc * A_acc.T + w3 * A_jerk * A_jerk.T
    Q = 2 * Q
    P = matrix(np.zeros((3*qp_size, 1)))
    for i in range(qp_size):
        P[3*i+1] = -2 * w1 * ref_speed

    # 等式约束 Aeq.T x = beq 这里有转置
    Aeq = matrix(np.zeros((3*qp_size, 2*qp_size-2)))
    beq = matrix(np.zeros((2*qp_size-2, 1)))
    dt = dp_points_t[1] - dp_points_t[0]
    Aeq_sub = np.array([[1,              0],
                       [dt,              1],
                        [(1/3)*dt**2, (1/2)*dt],
                        [-1,             0],
                        [0,             -1],
                        [(1/6)*dt**2, dt/2]])
    for i in range(qp_size - 1):
        Aeq[3*i : 3*i+5 +1, 2*i : 2*i+1 +1] = Aeq_sub


    # 不等式约束 A x <= b
    A = matrix(np.zeros((qp_size - 1, 3*qp_size)))
    b = matrix(np.zeros((qp_size - 1, 1)))
    for i in range(qp_size - 1):
        A[i, 3*i] = 1
        A[i, 3*i+3] = -1

    lb = matrix(np.zeros((3*qp_size, 1)))
    ub = lb

    # 凸空间
    for i in range(0, qp_size):
        if i == 0:
            # 起点约束
            lb[3*i] = start_state[1]
            lb[3*i+1] = start_state[2]
            lb[3*i+2] = start_state[3]
            ub[3*i] = lb[0]
            ub[3*i+1] = lb[1]
            ub[3*i+2] = lb[2]
        elif i == qp_size - 1:
            # 终点约束
            lb[3*i] = end_state[1]
            lb[3*i+1] = end_state[2]
            lb[3*i+2] = end_state[3]
            ub[3*i] = lb[3*i]
            ub[3*i+1] = lb[3*i+1]
            ub[3*i+2] = lb[3*i+2]
        else:
            # 中间点约束
            lb[3*i] = s_lb[i]
            ub[3*i] = s_ub[i]
            lb[3*i+1] = s_d_lb
            ub[3*i+1] = s_d_ub
            lb[3*i+2] = a_min
            ub[3*i+2] = a_max

    G = np.vstack((A, np.eye(3 * qp_size), -np.eye(3 * qp_size)))
    G = matrix(G)
    h = np.vstack((b, ub, -lb))
    h = matrix(h)

    Q_np = np.array(Q)
    # 计算特征值
    eigvals = np.linalg.eigvals(Q_np)

    if np.all(eigvals > 0):
        print("Q矩阵是正定的")
    elif np.all(eigvals >= 0):
        print("Q矩阵是半正定的")
    elif np.all(eigvals < 0):
        print("Q矩阵是负定的")
    else:
        print("Q矩阵既不是正定的，也不是半正定的，也不是负定的")

    G_np = np.array(G)
    # 计算矩阵的秩
    rank = np.linalg.matrix_rank(G_np)
    # 检查矩阵的秩是否等于其行数或列数
    if rank == min(G_np.shape):
        print("G矩阵的行或列是线性无关的")
    else:
        print("G矩阵的行或列是线性相关的")

    Aeq_np = np.array(Aeq.T)
    # 计算矩阵的秩
    rank = np.linalg.matrix_rank(Aeq_np)
    # 检查矩阵的秩是否等于其行数或列数
    if rank == min(Aeq_np.shape):
        print("Aeq矩阵的行或列是线性无关的")
    else:
        print("Aeq矩阵的行或列是线性相关的")

    # 求解QP问题
    options = {'show_progress': True, 'maxiters': 10000, 'abstol': 1e-9, 'reltol': 1e-9, 'refinement': 2}
    sol = solvers.qp(Q, P, G, h, Aeq.T, beq, options=options)    # 返回一个字典 'x'：一个cvxopt矩阵，包含了问题的解。这个矩阵的大小和你的问题的变量数量相同。
    print(sol)
    return sol['x']