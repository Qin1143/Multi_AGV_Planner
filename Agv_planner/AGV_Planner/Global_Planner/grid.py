#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import Tuple
import numpy as np

class Grid:

    def __init__(self, grid_size, static_obstacles):
        self.grid_size = grid_size
        self.minx, self.maxx, self.miny, self.maxy = self.calculate_boundaries(static_obstacles)
        self.grid = self.make_grid(grid_size, self.minx, self.maxx, self.miny, self.maxy)

    @staticmethod
    def calculate_boundaries(static_obstacles: np.ndarray) -> Tuple[int, int, int, int]:
        min_ = np.min(static_obstacles, axis=0)
        max_ = np.max(static_obstacles, axis=0)
        return min_[0], max_[0], min_[1], max_[1]

    @staticmethod
    def make_grid(grid_size: int, minx: int, maxx: int, miny: int, maxy: int) -> np.ndarray:
        # Calculate the size of the sides
        x_size = (maxx - minx) // grid_size
        y_size = (maxy - miny) // grid_size
        # Initialize the grid, assuming grid is 2D
        grid = np.zeros([y_size, x_size, 2], dtype=np.int32) # 创建一个形状为(y_size, x_size, 2)的全零数组grid，数据类型为int32，grid为三维，最后一个维度的大小为2。

        # Fill the grid in
        y = miny - grid_size / 2
        for i in range(y_size):
            y += grid_size
            x = minx - grid_size / 2
            for j in range(x_size):
                x += grid_size
                grid[i][j] = np.array([x, y])
        return grid

    '''
    Snap an arbitrary position to the center of the grid
    这个函数将一个任意位置(position)相对于网格的中心进行对齐，返回对齐后的位置。它通过计算给定位置距离网格的最小值与网格大小的整数商来确定对齐后的坐标。
    如果计算得到的坐标超出了网格的范围，则将其减1。最后，返回对齐后位置在网格中的坐标。
    '''
    def snap_to_grid(self, position: np.ndarray) -> np.ndarray:
        i = (position[1] - self.miny) // self.grid_size
        j = (position[0] - self.minx) // self.grid_size
        if i >= len(self.grid):
            i -= 1
        if j >= len(self.grid[0]):
            j -= 1
        return self.grid[i][j]