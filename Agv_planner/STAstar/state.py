#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
import numpy as np

class State:
    """
    状态类，表示机器人在迷宫中的状态

    :param pos: np.ndarray，机器人的位置坐标
    :param time: int，记录机器人已经消耗的时间
    :param g_score: int，记录机器人从起点到达当前位置的代价
    :param h_score: int，记录机器人从当前位置到达终点的估计代价

    """

    def __init__(self, pos: np.ndarray, time: int, g_score: int, h_score: int):
        self.pos = pos
        self.time = time
        self.g_score = g_score
        self.f_score = g_score + h_score

    def __hash__(self) -> int: # ‘__’代表私有函数
        """
        返回状态的哈希值

        :return: int，哈希值

        """

        concat = str(self.pos[0]) + str(self.pos[1]) + '0' + str(self.time)
        return int(concat)

    def pos_equal_to(self, pos: np.ndarray) -> bool:
        """
        判断状态的坐标位置是否与给定位置相等

        :param pos: np.ndarray，给定位置坐标
        :return: bool，是否相等

        """

        return np.array_equal(self.pos, pos)

    def __lt__(self, other: 'State') -> bool:
        """
        判断状态的f_score是否小于给定状态的f_score

        :param other: State，给定状态
        :return: bool，是否小于

        """

        return self.f_score < other.f_score

    def __eq__(self, other: 'State') -> bool:
        """
        判断状态的哈希值是否与给定状态的哈希值相等

        :param other: State，给定状态
        :return: bool，是否相等

        """

        return self.__hash__() == other.__hash__()

    def __str__(self):
        """
        返回状态的字符串表示形式

        :return: str，状态的字符串表示形式

        """

        return 'State(pos=[' + str(self.pos[0]) + ', ' + str(self.pos[1]) + '], ' \
               + 'time=' + str(self.time) + ', fscore=' + str(self.f_score) + ')'

    def __repr__(self):
        """
        返回状态的字符串表示形式

        :return: str，状态的字符串表示形式

        """

        return self.__str__()

