#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import Tuple
import numpy as np


class Agent:

    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int]):
        self.start = np.array(start)
        self.goal = np.array(goal)

    # Uniquely identify an agent with its start position
    def __hash__(self):  # 重写hash函数
        return int(str(self.start[0]) + str(self.start[1]))

    def __eq__(self, other: 'Agent'):  # 重写eq函数，eq函数用于判断两个对象是否相等
        return np.array_equal(self.start, other.start) and \
               np.array_equal(self.goal, other.goal)

    def __str__(self):  # 重写str函数，str函数用于返回对象的描述信息
        return str(self.start.tolist())

    def __repr__(self):  # 重写repr函数，repr函数用于返回对象的描述信息
        return self.__str__()
