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
    def __init__(self, starts, goals, mission_num, height, width, obs, paths: dict, corners_index: dict, name):
        self.starts, self.goals = starts, goals
        self.height, self.width, self.obs = height, width, obs
        self.paths = paths
        self.corners_index = corners_index
        self.name = name
        cmap = plt.get_cmap("viridis")
        self.mission_num = mission_num
        self.colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        self.multi_plot_2D(self.paths, self.name)

    def multi_plot_2D(self, paths, name):
        self.plot_grid(name)
        self.plot_static_paths(paths)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        plt.figure(figsize=(8, 6))
        for i in range(self.mission_num):
            plt.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8)
            plt.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8)
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_static_paths(self, paths):
        for i in range(self.mission_num):
            path_x = [point[0] for point in paths[i]]
            path_y = [point[1] for point in paths[i]]
            corners_x = [path_x[j] for j in self.corners_index[i]]
            corners_y = [path_y[j] for j in self.corners_index[i]]
            plt.plot(path_x, path_y, linewidth='3', color=self.colors[i], alpha=0.5)
            plt.plot(corners_x, corners_y, 'o', color=self.colors[i], alpha=1)
