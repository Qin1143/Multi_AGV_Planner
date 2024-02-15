"""
Plot tools 2D and 3D
@author: Ziqi Qin
"""
import math
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from create_map import get_map_data
import numpy as np

class Plotting:
    def __init__(self, starts, goals, mission_num, height, width, obs, paths: dict, paths_angle: dict,corners_index: dict,
                 DP_paths_all, QP_paths_all, S):
        self.starts, self.goals = starts, goals
        self.height, self.width, self.obs = height, width, obs
        self.paths = paths
        self.paths_angle = paths_angle
        self.corners_index = corners_index
        cmap = plt.get_cmap("viridis")
        self.mission_num = mission_num
        self.DP_paths_all = DP_paths_all
        self.QP_paths_all = QP_paths_all
        self.S = S
        self.colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        self.multi_plot_2D(self.paths)
        self.plot_show_DP_QP(self.DP_paths_all, self.QP_paths_all)

    def multi_plot_2D(self, paths):
        self.plot_grid()
        self.plot_static_paths(paths)
        plt.show()

    def plot_grid(self):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        plt.figure(figsize=(8, 6))
        for i in range(self.mission_num):
            plt.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8)
            plt.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8)
        plt.plot(obs_x, obs_y, "sk")
        plt.xlabel('X(m)')
        plt.ylabel('Y(M)')
        plt.title('Multi_AGV_Planner')
        plt.axis("equal")

    def plot_static_paths(self, paths):
        for i in range(self.mission_num):
            path_x = [point[0] for point in paths[i]]
            path_y = [point[1] for point in paths[i]]
            corners_x = [path_x[j] for j in self.corners_index[i]]
            corners_y = [path_y[j] for j in self.corners_index[i]]
            plt.plot(path_x, path_y, linewidth='3', color=self.colors[i], alpha=0.5)
            plt.plot(corners_x, corners_y, 'o', color=self.colors[i], alpha=1)

            # for j in range(len(path_x) - 1):  # 每隔10个点绘制一个箭头
            #     if j % 10 == 0:  # 只在每隔10个点处绘制箭头
            #         dx = path_x[j + 1] - path_x[j]
            #         dy = path_y[j + 1] - path_y[j]
            #         plt.quiver(path_x[j], path_y[j], dx, dy, angles='xy', scale_units='xy', scale=1,
            #                    color=self.colors[i])

    def plot_show_DP_QP(self, DP_paths_all, QP_paths_all):
        # 展示轨迹
        cmap = plt.get_cmap("viridis")
        colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        # print(self.mission_num)
        for i in range(self.mission_num):
            plt.figure()
            Tims = np.arange(len(self.paths[i]))
            plt.plot(Tims, self.S[i], colors[i])
            plt.plot(DP_paths_all[i][:, 1], DP_paths_all[i][:, 0], colors[i])
            plt.plot(QP_paths_all[i][:, 1], QP_paths_all[i][:, 0], colors[i])
            # 图例
            plt.legend(['STAstar', 'DP', 'QP'])
            plt.xlabel('T(s)')
            plt.ylabel('S(m)')
            plt.title('ST_Figure_mission_' + str(i))
            plt.show()