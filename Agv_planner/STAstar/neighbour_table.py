#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
import numpy as np

class NeighbourTable:
    #             Current  Right    Left    Down     Up
    directions = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
    # Uncomment the line below for 9 directions, this might slow things down a lot
    # directions = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def __init__(self, grid: np.ndarray):
        # 初始化函数，用于构建邻居表
        # 参数：
        #   - grid: 输入的网格，是一个二维numpy数组
        # 返回值：无
        dimy, dimx = len(grid), len(grid[0])
        table = dict()
        for i in range(dimy):
            for j in range(dimx):
                neighbours = []
                # 遍历所有可能的方向
                for dx, dy in self.directions:
                    y, x = i + dy, j + dx,
                    if x >= 0 and x < dimx and y >= 0 and y < dimy:
                        # 判断下一个位置是否在网格范围内
                        neighbours.append(grid[y][x])
                table[self.hash(grid[i][j])] = np.array(neighbours)
        self.table = table


    def lookup(self, position: np.ndarray) -> np.ndarray:
        """
        根据给定的位置查找对应的数据。

        参数：
        position (np.ndarray): 一个包含位置信息的数组。

        返回值：
        np.ndarray: 包含查找结果的数组。
        """
        return self.table[self.hash(position)]


    @staticmethod
    def hash(grid_pos: np.ndarray) -> tuple:
        """
        将给定的网格位置转换为哈希值。

        参数：
        grid_pos (np.ndarray): 网格位置，表示为一个包含多个坐标值的数组。

        返回：
        int: 转换后的哈希值。

        """
        return tuple(grid_pos)


if __name__ == '__main__':
    grid = np.array([[[15,5],[15,6],[15,7],[15,8],[15,9]],
            [[16,5],[16,6],[16,7],[16,8],[16,9]],
            [[17,5],[17,6],[17,7],[17,8],[17,9]],
            [[18,5],[18,6],[18,7],[18,8],[18,9]],
            [[19,5],[19,6],[19,7],[19,8],[19,9]]])
    nt = NeighbourTable(grid)
    print(nt.lookup(np.array([16,7])))