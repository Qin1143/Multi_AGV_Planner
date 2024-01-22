"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import time
import numpy as np
from Astar import AStar
import matplotlib.pyplot as plt
from turtledemo.clock import jump

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from path_quality import PathQuality as pq


class AgvJpsAstar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.Env = env.Env(use_benchmark=True)
        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.matrix = np.zeros((self.Env.y_range, self.Env.x_range))
        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                if (i, j) in self.obs:
                    self.matrix[j][i] = 1
        self.OPEN = []
        self.CLOSED = []
        self.PARENT = dict()
        self.g = dict()

    def direction(self, cX, cY, pX, pY):  # 返回当前节点和父节点的方向
        dX = int(math.copysign(1, cX - pX))  # math.copysign() 方法返回数字大小和符号，即将第二个参数的符号传给第一个参数。
        dY = int(math.copysign(1, cY - pY))
        if cX - pX == 0:
            dX = 0
        if cY - pY == 0:
            dY = 0
        return (dX, dY)

    def blocked(self, cX, cY, dX, dY, matrix):  # 判断当前节点是否为障碍物
        """
        :param cX: Current X
        :param cY: Current Y
        :param dX: Expand distance in X
        :param dY: Expand distance in Y
        :param matrix: Map
        :return: True: is collision / False: not collision
        """
        # print("Start blocked...")
        if cX + dX < 0 or cX + dX >= matrix.shape[1]:  # x轴越界
            return True
        elif cY + dY < 0 or cY + dY >= matrix.shape[0]:  # y轴越界
            return True
        else:
            if matrix[cY + dY][cX + dX] == 1:
                return True
        return False

    def dblock(self, cX, cY, dX, dY, matrix):
        if matrix[cY][cX - dX] == 1 and matrix[cY - dY][cX] == 1:
            return True
        else:
            return False

    def nodeNeighbours(self, cX, cY, parent, matrix):
        # print("Start nodeNeighbours...")
        neighbours = []
        motions = self.u_set
        if parent == (cX, cY):  # 判断是否是初始节点
            for x, y in motions:
                if not self.blocked(cX, cY, x, y, matrix):
                    neighbours.append((cX + x, cY + y))
            return neighbours
        dX, dY = self.direction(cX, cY, parent[0], parent[1])

        if dX != 0:
            if not self.blocked(cX, cY, dX, 0, matrix):
                neighbours.append((cX + dX, cY))
            if not self.blocked(cX, cY, dX, -1, matrix):
                neighbours.append((cX, cY - 1))
            if not self.blocked(cX, cY, dX, 1, matrix):
                neighbours.append((cX, cY + 1))
        if dY != 0:
            if not self.blocked(cX, cY, 0, dY, matrix):
                neighbours.append((cX, cY + dY))
            if not self.blocked(cX, cY, -1, dY, matrix):
                neighbours.append((cX - 1, cY))
            if not self.blocked(cX, cY, 1, dY, matrix):
                neighbours.append((cX + 1, cY))

        return neighbours

    def jump(self, cX, cY, dX, dY, matrix, goal, direct_search=True):
        # print("Start jump...")
        nX = cX + dX
        nY = cY + dY
        if self.blocked(nX, nY, 0, 0, matrix):
            # print("Blocked", (nX, nY))
            return None

        if (nX, nY) == goal:
            # print("Find the goal!", (nX, nY))
            return (nX, nY)

        oX = nX
        oY = nY
        # print((oX, oY))

        if dX != 0:
            while True:
                if (
                        (not self.blocked(oX, nY, 0, 1, matrix) and self.blocked(oX, nY, -dX, 1, matrix))  # 判断强迫节点
                        or
                        (not self.blocked(oX, nY, 0, -1, matrix) and self.blocked(oX, nY, -dX, -1, matrix))  # 判断强迫节点
                ):
                    # print("Find the jump point defined by forcing neighbors!")
                    return (oX, nY)

                if direct_search:
                    if self.jump(oX, nY, 0, 1, matrix, goal, False) != None or self.jump(oX, nY, 0, -1, matrix, goal,
                                                                                         False) != None:
                        return (oX, nY)

                oX += dX

                if self.blocked(oX, nY, 0, 0, matrix):
                    return None

                if (oX, nY) == goal:
                    return (oX, nY)

        else:  # dY != 0
            while True:
                if (
                        (not self.blocked(nX, oY, 1, 0, matrix) and self.blocked(nX, oY, 1, -dY, matrix))  # 判断强迫节点
                        or
                        (not self.blocked(nX, oY, -1, 0, matrix) and self.blocked(nX, oY, -1, -dY, matrix))  # 判断强迫节点
                ):
                    # print("Find the jump point defined by forcing neighbors!")
                    return (nX, oY)

                if direct_search:
                    if self.jump(nX, oY, 1, 0, matrix, goal, False) != None or self.jump(nX, oY, -1, 0, matrix, goal,
                                                                                         False) != None:
                        return (nX, oY)

                oY += dY

                if self.blocked(nX, oY, 0, 0, matrix):
                    return None

                if (nX, oY) == goal:
                    return (nX, oY)
        # if direct_search:
        #     return self.jump(nX, nY, dX, dY, matrix, goal, True)
        # else:
        #     return self.jump(nX, nY, dX, dY, matrix, goal, False)

    def identifySuccessors(self, cX, cY, PARENT, matrix, goal):
        # print("Start identifySuccessors...")
        successors = []
        neighbours = self.nodeNeighbours(cX, cY, PARENT[(cX, cY)], matrix)
        # print("neighbours:", neighbours)

        for cell in neighbours:
            dX = cell[0] - cX
            dY = cell[1] - cY

            jumpPoint = self.jump(cX, cY, dX, dY, matrix, goal)
            # print("jumpPoint:", jumpPoint)

            if jumpPoint != None:
                successors.append(jumpPoint)
                # print("successors:", successors)

        return successors

    def searching(self):
        # print("Start searching...")
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start, self.PARENT), self.s_start))
        start_time = time.time()
        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            # print("Min f_value:", s)  # 弹出OPEN列表中f值最小的状态s，状态s的f数值为f_value(s)，缺省，暂不需要
            self.CLOSED.append(s)
            if s == self.s_goal:
                # print("Find the goal!")
                break
            successers = self.identifySuccessors(s[0], s[1], self.PARENT, self.matrix, self.s_goal)
            # print("successers in searching:", successers)
            for successer in successers:
                jumpPoint = successer
                # print("jumpPoint in searching:", jumpPoint)

                if (jumpPoint in self.CLOSED):
                    continue

                hchoice = 2
                tentative_g_score = self.g[s] + self.lenght(s, jumpPoint, hchoice)

                if tentative_g_score < self.g.get(jumpPoint, 0) or jumpPoint not in [j[1] for j in self.OPEN]:
                    self.PARENT[jumpPoint] = s
                    self.g[jumpPoint] = tentative_g_score
                    f_jumpPoint = self.f_value(jumpPoint, self.PARENT)
                    heapq.heappush(self.OPEN, (f_jumpPoint, jumpPoint))

        end_time = time.time()
        return self.extract_path(self.PARENT), self.CLOSED, round(end_time - start_time, 6)

    def lenght(self, current, jumppoint, hchoice):
        # print("Start lenght...")
        dX, dY = self.direction(current[0], current[1], jumppoint[0], jumppoint[1])
        dX = math.fabs(dX)
        dY = math.fabs(dY)
        lX = math.fabs(current[0] - jumppoint[0])
        lY = math.fabs(current[1] - jumppoint[1])
        if hchoice == 1:
            if dX != 0 and dY != 0:
                lenght = lX * 14
                return lenght
            else:
                lenght = (dX * lX + dY * lY) * 10
                return lenght
        if hchoice == 2:
            return math.sqrt(
                (current[0] - jumppoint[0]) ** 2 + (current[1] - jumppoint[1]) ** 2
            )

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

    def f_value(self, s, PARENT):
        """
        f = g + w1 * h! + w2 * h2. (g: Cost to come, h1: heuristic value, h2: heuristic value)
        :param s: current state
        :return: f
        """
        # return self.g[s] + self.heuristic(s)
        [w1, w2] = [2, 2]
        s_l = PARENT[s]
        s_ll = PARENT[s_l]
        if s_l == s or s_ll == s_l:
            return self.g[s] + self.heuristic_1st(s)
        else:
            return self.g[s] + w1 * self.heuristic_1st(s) + w2 * self.heuristic_2nd(s_ll, s_l, s)

    def heuristic(self, a, hchoice=2):
        b = self.s_goal
        if hchoice == 1:
            xdist = math.fabs(b[0] - a[0])
            ydist = math.fabs(b[1] - a[1])
            if xdist > ydist:
                return 14 * ydist + 10 * (xdist - ydist)
            else:
                return 14 * xdist + 10 * (ydist - xdist)
        if hchoice == 2:
            return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def heuristic_1st(self, s):
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

    def heuristic_2nd(self, s_ll, s_l, s):
        # Create vectors [s_last to s] and [s to s_next]
        vector_1 = [s_l[0] - s_ll[0], s_l[1] - s_ll[1]]
        vector_2 = [s[0] - s_l[0], s[1] - s_l[1]]

        # Calculate the dot product
        dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]

        # Calculate the magnitude of each vector
        magnitude_1 = math.sqrt(vector_1[0] ** 2 + vector_1[1] ** 2)
        magnitude_2 = math.sqrt(vector_2[0] ** 2 + vector_2[1] ** 2)

        # Avoid division by zero
        if magnitude_1 * magnitude_2 == 0:
            return 0

        # Calculate and return the angle (in degrees)
        angle = dot_product / (magnitude_1 * magnitude_2)
        return abs(angle - 1)


class AgvAStar:
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

        self.PARENT[self.s_start] = self.s_start  # 父节点字典赋值
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        # 将一个元组添加到堆栈self.OPEN中
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start, self.PARENT), self.s_start))

        start_time = time.time()
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
                    heapq.heappush(self.OPEN, (self.f_value(s_n, self.PARENT), s_n))  # 将状态s_n和其f值推入OPEN列表中

        end_time = time.time()
        return self.extract_path(self.PARENT), self.CLOSED, round(end_time - start_time, 6)

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

    def f_value(self, s, PARENT):
        """
        f = g + w1 * h! + w2 * h2. (g: Cost to come, h1: heuristic value, h2: heuristic value)
        :param s: current state
        :return: f
        """
        # [w1, w2] = [1, 1]
        [w1, w2] = [2, 2]
        s_l = PARENT[s]
        s_ll = PARENT[s_l]
        if s_l == s or s_ll == s_l:
            return self.g[s] + self.heuristic_1st(s)
        else:
            return self.g[s] + w1 * self.heuristic_1st(s) + w2 * self.heuristic_2nd(s_ll, s_l, s)

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

    def heuristic_1st(self, s):
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

    def heuristic_2nd(self, s_ll, s_l, s):
        # Create vectors [s_last to s] and [s to s_next]
        vector_1 = [s_l[0] - s_ll[0], s_l[1] - s_ll[1]]
        vector_2 = [s[0] - s_l[0], s[1] - s_l[1]]

        # Calculate the dot product
        dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]

        # Calculate the magnitude of each vector
        magnitude_1 = math.sqrt(vector_1[0] ** 2 + vector_1[1] ** 2)
        magnitude_2 = math.sqrt(vector_2[0] ** 2 + vector_2[1] ** 2)

        # Avoid division by zero
        if magnitude_1 * magnitude_2 == 0:
            return 0

        # Calculate and return the angle (in degrees)
        angle = dot_product / (magnitude_1 * magnitude_2)
        return abs(angle - 1)


def main():
    '''
    在此处修改注释，使用不同的AGV全局规划算法。
    '''
    s_start = (5, 5)
    s_goal = (160, 80)

    astar_m = AStar(s_start, s_goal, "manhattan")
    astar_e = AStar(s_start, s_goal, "euclidean")
    agv_astar = AgvAStar(s_start, s_goal, "manhattan")
    agv_jps_astar = AgvJpsAstar(s_start, s_goal, "manhattan")

    # print(agv_jps_astar.blocked(25, 9, 1, 0, agv_jps_astar.matrix))
    # print(agv_jps_astar.blocked(25, 9, 1, -1, agv_jps_astar.matrix))
    # print(agv_jps_astar.matrix[9][26])
    # print(agv_jps_astar.matrix[8][26])

    # plt.imshow(agv_jps_astar.matrix, cmap='binary')  # 使用二值颜色映射
    # plt.colorbar()
    # plt.show()

    plot = plotting.Plotting(s_start, s_goal)

    path, visited, time = agv_astar.searching()
    # path, visited, time = agv_jps_astar.searching()

    print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
          pq(path).num_turn, 'Num_close:', len(visited))
    # print(visited)

    # plot.static_plot(path, visited, "AGV_JPS_A*")  # static plot
    # plot.static_plot(path, visited, "AGV_A*")  # static plot

    plot.animation(path, visited, "AGV_A*")  # animation


if __name__ == '__main__':
    main()
