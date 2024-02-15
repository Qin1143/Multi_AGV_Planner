"""
@Author: Ziqi Qin
Email: qinziqi@tju.edu.cn
"""
from typing import Tuple, List, Dict, Set
import matplotlib.pyplot as plt
# from .DP import dp_planner
from .DP_fun import dp_planner
from .OSQP import qp_planner_osqp as qp_planner
import numpy as np

class Traj_Planner:
    def __init__(self, paths: dict, dy_obs: dict, mission_num: int, corners: dict, dp_vertex_constraints: dict, up_dp_bound: dict, low_dp_bound: dict):
        self.paths = paths
        self.S = dict()
        self.dy_obs = dy_obs
        self.mission_num = mission_num
        self.corners = corners

        self.DP_paths_s = dict()
        self.DP_paths_t = dict()
        self.DP_vertex_constraints = dict()
        self.DP_up_bound = dict()
        self.DP_low_bound = dict()

        self.QP_paths_s = dict()
        self.QP_paths_t = dict()
        self.path_sub = dict()
        [self.v_max, self.v_min, self.a_max, self.a_min] = [3.0, 0.0, 2.0, -2.0]


        # 将每个轨迹按照转折点分段
        for i in range(self.mission_num):

            self.S[i] = np.zeros(len(self.paths[i]))
            for k in range(1, len(self.paths[i])):
                if np.array_equal(self.paths[i][k], self.paths[i][k-1]):
                    self.S[i][k] = self.S[i][k-1]
                else:
                    self.S[i][k] = self.S[i][k-1]+1

            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    self.path_sub[(i, j)] = self.paths[i][0:self.corners[i][j] +1]
                    self.DP_vertex_constraints[(i, j)] = dp_vertex_constraints[i][0:self.corners[i][j] +1]
                    self.DP_up_bound[(i, j)] = up_dp_bound[i][0:self.corners[i][j] +1]
                    self.DP_low_bound[(i, j)] = low_dp_bound[i][0:self.corners[i][j] +1]
                elif j == len(self.corners[i])/2:
                    self.path_sub[(i, j)] = self.paths[i][self.corners[i][j*2-1]:]
                    self.DP_vertex_constraints[(i, j)] = dp_vertex_constraints[i][self.corners[i][j*2-1]:]
                    self.DP_up_bound[(i, j)] = up_dp_bound[i][self.corners[i][j*2-1]:]
                    self.DP_low_bound[(i, j)] = low_dp_bound[i][self.corners[i][j*2-1]:]
                else:
                    self.path_sub[(i, j)] = self.paths[i][self.corners[i][j*2-1]:self.corners[i][j*2] +1]
                    self.DP_vertex_constraints[(i, j)] = dp_vertex_constraints[i][self.corners[i][j*2-1]:self.corners[i][j*2] +1]
                    self.DP_up_bound[(i, j)] = up_dp_bound[i][self.corners[i][j*2-1]:self.corners[i][j*2] +1]
                    self.DP_low_bound[(i, j)] = low_dp_bound[i][self.corners[i][j*2-1]:self.corners[i][j*2] +1]

        for i in range(self.mission_num):
            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    dy_obs_in, dy_obs_out = self.vertex_cons_2_st(self.DP_vertex_constraints[(i, j)])
                    self.DP_paths_s[(i, j)], self.DP_paths_t[(i, j)], end_state = dp_planner(path=self.path_sub[(i, j)], dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out,
                                                       v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
                    start_state = (self.DP_paths_t[(i, j)][0, 0], self.DP_paths_s[(i, j)][0, 0], 0, 0)
                    end_state = (end_state[0], end_state[1], 0, 0)
                    osqp_result = qp_planner(self.DP_paths_t[(i, j)], self.DP_paths_s[(i, j)], self.v_max, self.v_min, start_state=start_state,
                                          end_state=end_state, dy_obs_in=[], dy_obs_out=[])
                    self.QP_paths_s[(i, j)], self.QP_paths_t[(i, j)] = self.qp_result2fun(osqp_result, self.DP_paths_t[(i, j)])

                elif j == len(self.corners[i])/2:
                    dy_obs_in, dy_obs_out = self.vertex_cons_2_st(self.DP_vertex_constraints[(i, j)])
                    self.DP_paths_s[(i, j)], self.DP_paths_t[(i, j)], end_state = dp_planner(path=self.path_sub[(i, j)], dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out,
                                                       v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
                    start_state = (self.DP_paths_t[(i, j)][0, 0], self.DP_paths_s[(i, j)][0, 0], 0, 0)
                    end_state = (end_state[0], end_state[1], 0, 0)
                    osqp_result = qp_planner(self.DP_paths_t[(i, j)], self.DP_paths_s[(i, j)], self.v_max, self.v_min, start_state=start_state,
                                          end_state=end_state, dy_obs_in=[], dy_obs_out=[])
                    self.QP_paths_s[(i, j)], self.QP_paths_t[(i, j)] = self.qp_result2fun(osqp_result, self.DP_paths_t[(i, j)])

                else:
                    dy_obs_in, dy_obs_out = self.vertex_cons_2_st(self.DP_vertex_constraints[(i, j)])
                    self.DP_paths_s[(i, j)], self.DP_paths_t[(i, j)], end_state = dp_planner(path=self.path_sub[(i, j)], dy_obs_in=dy_obs_in, dy_obs_out=dy_obs_out,
                                                       v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
                    start_state = (self.DP_paths_t[(i, j)][0, 0], self.DP_paths_s[(i, j)][0, 0], 0, 0)
                    end_state = (end_state[0], end_state[1], 0, 0)
                    osqp_result = qp_planner(self.DP_paths_t[(i, j)], self.DP_paths_s[(i, j)], self.v_max, self.v_min, start_state=start_state,
                                          end_state=end_state, dy_obs_in=[], dy_obs_out=[])
                    self.QP_paths_s[(i, j)], self.QP_paths_t[(i, j)] = self.qp_result2fun(osqp_result, self.DP_paths_t[(i, j)])

        # print(self.path_sub)
        self.DP_paths_all = self.connect_dp_paths(self.DP_paths_s, self.DP_paths_t)
        self.QP_paths_all = self.connect_qp_paths(self.QP_paths_s, self.QP_paths_t)
        print(self.QP_paths_all[8])
        # self.plot_show(self.DP_paths_all, self.QP_paths_all)

    def vertex_cons_2_st(self, vertex_cons: List[Tuple[float, float, float, float]]) -> Tuple[List, List]:
        dy_obs_in = []
        dy_obs_out = []
        for index, k in enumerate(vertex_cons[:-1]):
            if index == 0 and vertex_cons[index] == 1:
                dy_obs_in.append((index, index))
            elif index == len(vertex_cons) - 2 and vertex_cons[index+1] == 1:
                dy_obs_out.append((index, index))
            elif k == 0 and vertex_cons[index+1] == 1:
                dy_obs_in.append((index+1, index+1))
            elif k == 1 and vertex_cons[index+1] == 0:
                dy_obs_out.append((index, index))
            else:
                continue
        print('dy_obs_in:', dy_obs_in, 'dy_obs_out:', dy_obs_out)
        return dy_obs_in, dy_obs_out



    def qp_result2fun(self, osqp_result, dp_points_t):
        # 将OSQP的结果转化为函数
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

    def connect_dp_paths(self, DP_paths_s: dict, DP_paths_t: dict):
        DP_paths_all = dict()
        for i in range(self.mission_num):
            DP_paths_all[i] = np.array([])
            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    DP = np.column_stack((DP_paths_s[(i, j)], DP_paths_t[(i, j)]))
                    DP_paths_all[i] = DP
                else:
                    DP = np.column_stack((DP_paths_s[(i, j)], DP_paths_t[(i, j)])) + DP_paths_all[i][-1] + [0, 1]
                    DP_paths_all[i] = np.concatenate((DP_paths_all[i], DP), axis=0)
        return DP_paths_all

    def connect_qp_paths(self, QP_paths_s: dict, QP_paths_t: dict):
        QP_paths_all = dict()
        for i in range(self.mission_num):
            QP_paths_all[i] = np.array([])
            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    QP = np.column_stack((QP_paths_s[(i, j)], QP_paths_t[(i, j)]))
                    QP_paths_all[i] = QP
                    t = np.linspace(QP_paths_all[i][-1, 1], QP_paths_all[i][-1, 1] + 1, 100)
                    s = np.ones(100) * QP_paths_all[i][-1, 0]
                    gap = np.column_stack((s, t))
                    QP_paths_all[i] = np.concatenate((QP_paths_all[i], gap), axis=0)
                elif j == len(self.corners[i])/2:
                    QP = np.column_stack((QP_paths_s[(i, j)], QP_paths_t[(i, j)])) + QP_paths_all[i][-1]
                    QP_paths_all[i] = np.concatenate((QP_paths_all[i], QP), axis=0)
                else:
                    QP = np.column_stack((QP_paths_s[(i, j)], QP_paths_t[(i, j)])) + QP_paths_all[i][-1]
                    QP_paths_all[i] = np.concatenate((QP_paths_all[i], QP), axis=0)
                    t = np.linspace(QP_paths_all[i][-1, 1], QP_paths_all[i][-1, 1] + 1, 100)
                    s = np.ones(100) * QP_paths_all[i][-1, 0]
                    gap = np.column_stack((s, t))
                    QP_paths_all[i] = np.concatenate((QP_paths_all[i], gap), axis=0)
        return QP_paths_all

    def plot_show(self, DP_paths_all, QP_paths_all):
        # 展示轨迹
        plt.figure()
        cmap = plt.get_cmap("viridis")
        colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        # print(self.mission_num)
        for i in range(self.mission_num):
            # print(DP_paths_all[i].shape)
            # print(QP_all[i].shape)
            # print(self.S[i].shape)
            # print(len(self.paths[i]))
            Tims = np.arange(len(self.paths[i]))
            plt.plot(Tims, self.S[i], colors[i])
            plt.plot(Tims, DP_paths_all[i][:, 0], colors[i])
            plt.plot(QP_paths_all[i][:, 1], QP_paths_all[i][:, 0], colors[i])
            plt.xlabel('T(s)')
            plt.ylabel('S(m)')
            plt.title('ST_Figure_mission_' + str(i))
            plt.show()