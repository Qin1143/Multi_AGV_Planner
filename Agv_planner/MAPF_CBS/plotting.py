"""
Plot tools 2D and 3D
@author: Ziqi Qin
"""

import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from Agv_planner.create_map import get_map_data
import numpy as np

class Plotting:
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.height, self.width, self.obs = get_map_data()
        cmap = plt.get_cmap("viridis")
        self.num_iterations = len(xI)
        self.colors = [cmap(i) for i in np.linspace(0, 1, self.num_iterations)]

    def multi_plot_2D(self, path, name):
        self.plot_grid(name)
        self.plot_static_paths(path)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        plt.figure(figsize=(8, 6))
        for i in range(len(self.xI)):
            plt.plot(self.xI[i][0], self.xI[i][1], color=self.colors[i], marker='o', markersize=5)
            plt.plot(self.xG[i][0], self.xG[i][1], color=self.colors[i], marker='^', markersize=5)
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_static_paths(self, path):
        for i in range(self.num_iterations):
            path_x = [point[0] for point in path[i]]
            path_y = [point[1] for point in path[i]]
            plt.plot(path_x, path_y, linewidth='3', color=self.colors[i])
