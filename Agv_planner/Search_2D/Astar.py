"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from path_quality import PathQuality as pq


class AStar:
    """
    AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env(use_benchmark=True)  # class Env, regenerate can be True or False

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent 字典类型
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """
        start_time = time.time()
        self.PARENT[self.s_start] = self.s_start # 父节点字典赋值
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        # 将一个元组添加到堆栈self.OPEN中
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))


        while self.OPEN:  # 当OPEN列表不为空时继续循环
            _, s = heapq.heappop(self.OPEN)  # 弹出OPEN列表中f值最小的状态s，状态s的f数值为f_value(s)，缺省，暂不需要
            self.CLOSED.append(s)  # 将状态s添加到CLOSED列表中

            if s == self.s_goal:  # 如果状态s等于目标状态GOAL，满足停止条件
                break  # 跳出循环

            for s_n in self.get_neighbor(s):  # 遍历状态s的邻居状态s_n
                new_cost = self.g[s] + self.cost(s, s_n)  # 计算从起点到状态s_n的总花费new_cost，cost中包含碰撞检测

                if new_cost == math.inf or s_n in self.CLOSED:  # 如果s_n与障碍物冲突或者状态s_n在CLOSED列表中，跳过
                    continue
                # 更新状态s_n的总花费 初次探索的s_n或在open_list中的s_n
                if s_n not in self.g:  # 如果状态s_n不在g字典中，即初次探索s_n
                    self.g[s_n] = math.inf  # 将状态s_n的总花费设置为无穷大

                if new_cost < self.g[s_n]:  # 如果new_cost小于当前状态s_n的总花费
                    self.g[s_n] = new_cost  # 更新状态s_n的总花费为new_cost
                    self.PARENT[s_n] = s  # 更新状态s_n的父节点为状态s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))  # 将状态s_n和其f值推入OPEN列表中

        end_time = time.time()
        return self.extract_path(self.PARENT), self.CLOSED, round(end_time - start_time, 6)

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """
        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:  # 如果起始点和终点的横纵坐标都不相等
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:  # 如果起始点和终点的横纵坐标差值相等
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 设定s1为起始点和终点横纵坐标的最小值
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 设定s2为起始点和终点横纵坐标的最大值
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))  # 设定s1为起始点和终点横纵坐标的最小值和最大值
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))  # 设定s2为起始点和终点横纵坐标的最大值和最小值

            if s1 in self.obs or s2 in self.obs:  # 如果s1或s2在self.obs中
                return True


        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

def main():
    s_start = (5, 5)
    s_goal = (160, 80)

    astar = AStar(s_start, s_goal, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)

    path, visited, time = astar.searching()
    print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
          pq(path).num_turn, 'Num_close:', len(visited))
    # print(visited)

    plot.static_plot(path, visited, "A*")  # static plot
    # plot.animation(path, visited, "A*")  # animation

    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
