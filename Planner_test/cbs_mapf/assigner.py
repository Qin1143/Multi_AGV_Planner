#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import List, Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

from .agent import Agent


# Hungarian algorithm for global minimal cost
def min_cost(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert (len(starts) == len(goals))

    sqdist = lambda x, y: (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2
    cost_vec = []
    for start in starts:
        for goal in goals:
            cost_vec.append(sqdist(start, goal))
    n = len(starts)
    cost_mtx = np.array(cost_vec).reshape((n, n))
    row_ind, col_ind = linear_sum_assignment(cost_mtx)
    agents = []
    for i, start in enumerate(starts):
        agents.append(Agent(start, goals[col_ind[i]]))
    return agents


# Greedily choosing closest distance
def greedy_assign(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    # 确认starts和goals的长度是否相等，如果starts和goals的长度不相等，assert语句会抛出AssertionError异常，程序会终止执行并输出错误信息。
    assert (len(starts) == len(goals))

    # 创建一个目标集合，将所有目标存储为集合并返回一个集合对象
    goal_set = set(goal for goal in goals)

    # 定义一个lambda函数，用于计算两个点之间的欧几里得距离
    sqdist = lambda x, y: (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2

    # 创建一个空列表来存储智能体对象
    agents = []

    # 遍历所有起始点
    for start in starts:
        # 初始化最近距离为正无穷大，最近目标为空
        closest = float('inf')
        closest_goal = None

        # 遍历目标集合中的每个目标
        for goal in goal_set:
            # 计算起始点和目标之间的欧几里得距离
            d = sqdist(start, goal)

            # 如果距离小于最近距离，更新最近距离和最近目标
            if d < closest:
                closest = d
                closest_goal = goal

        # 从目标集合中移除最近目标
        goal_set.remove(closest_goal)

        # 将最近目标和起始点作为参数创建一个智能体对象，并将其添加到智能体列表中
        agents.append(Agent(start, closest_goal))

    # 返回智能体列表
    return agents


def no_assign(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert (len(starts) == len(goals))

    agents = []
    i = 0
    for start in starts:
        agents.append(Agent(start, goals[i]))
        i += 1
    return agents