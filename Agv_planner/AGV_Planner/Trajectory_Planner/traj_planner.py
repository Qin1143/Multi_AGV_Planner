"""
@Author: Ziqi Qin
Email: qinziqi@tju.edu.cn
"""
from typing import Tuple, List, Dict, Set
from .DP import dp_planner
from .OSQP import qp_planner_osqp as qp_planner
import numpy as np

class Traj_Planner:
    def __init__(self, paths: dict, dy_obs: dict, mission_num: int, corners: dict):
        self.paths = paths
        self.dy_obs = dy_obs
        self.mission_num = mission_num
        self.corners = corners
        self.DP_paths = dict()
        self.QP_paths = dict()
        self.path_sub = dict()
        [self.v_max, self.v_min, self.a_max, self.a_min] = [3.0, 0.0, 2.0, -2.0]
        # 将每个轨迹按照转折点分段
        for i in range(self.mission_num):
            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    self.path_sub[(i, j)] = self.paths[i][0:self.corners[i][j] +1]
                elif j == len(self.corners[i])/2:
                    self.path_sub[(i, j)] = self.paths[i][self.corners[i][j*2-1]:]
                else:
                    self.path_sub[(i, j)] = self.paths[i][self.corners[i][j*2-1]:self.corners[i][j*2] +1]

        for i in range(self.mission_num):
            for j in range(len(self.corners[i])//2 + 1):
                if j == 0:
                    self.DP_paths[(i, j)] = dp_planner(path=self.path_sub[(i, j)], path_start_time=0, path_end_time=self.corners[i][j]+1,
                                                       dy_obs=self.dy_obs, v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
                elif j == len(self.corners[i])/2:
                    self.DP_paths[(i, j)] = dp_planner(path=self.path_sub[(i, j)], path_start_time=self.corners[i][j*2-1]+1, path_end_time=len(self.paths[i]),
                                                       dy_obs=self.dy_obs, v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
                else:
                    self.DP_paths[(i, j)] = dp_planner(path=self.path_sub[(i, j)], path_start_time=self.corners[i][j*2-1]+1, path_end_time=self.corners[i][j*2]+1,
                                                       dy_obs=self.dy_obs, v_max=self.v_max, v_min=self.v_min, a_max=self.a_max, a_min=self.a_min)
        # print(self.path_sub)
