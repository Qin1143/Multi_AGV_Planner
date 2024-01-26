#!/usr/bin/env python3
'''
Author: Ziqi Qin
Email: gavinsweden@gmail.com
'''
from typing import Tuple, List, Dict, Set
from heapq import heappush, heappop
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
import math
from .neighbour_table import NeighbourTable
from .grid import Grid
from .state import State


class Planner:
    def __init__(self, grid_size: int,
                       robot_radius: float,
                       static_obstacles: List[Tuple[int, int]]):

        self.grid_size = grid_size
        self.robot_radius = robot_radius
        np_static_obstacles = np.array(static_obstacles)
        # 它用于创建k维树，以便从一组点快速查找最近邻。该树可以查询任何给定点的最近邻居（也称为空间查询）。
        self.static_obstacles = KDTree(np_static_obstacles)

        # Make the grid according to the grid size
        self.grid = Grid(grid_size, np_static_obstacles)
        # Make a lookup table for looking up neighbours of a grid
        self.neighbour_table = NeighbourTable(self.grid.grid)

        self.dynamic_obstacles = None


    '''
    An admissible and consistent heuristic for A*
    '''

    '''
    @staticmethod是Python中的一个内置装饰器，用于定义一个类的静态方法。
    静态方法不接收任何特定的实例或类的引用，它绑定到类而不是类的实例。
    '''
    @staticmethod
    def h(start: np.ndarray, goal: np.ndarray, s_l: State = None, s_ll: State = None) -> int:
        [w1, w2] = [2, 2]
        h1 = np.linalg.norm(start-goal, 1)
        if s_l is None or s_ll is None:
            return int(h1)  # L1 norm 返回起点到终点的曼哈顿距离（L1范数）
        else:
            h2 = Planner.h2(s_ll.pos, s_l.pos, start)
            return int(w1*h1 + w2*h2)
    @staticmethod
    def h2(s_ll, s_l, s):
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
        angle = math.acos(angle)
        return abs(angle)

    @staticmethod
    def l2(start: np.ndarray, goal: np.ndarray) -> int:
        return int(np.linalg.norm(start-goal, 2))  # L2 norm 返回起点到终点的欧几里得距离（L2范数）

    '''
    Check whether the nearest static obstacle is within radius
    '''
    def safe_static(self, grid_pos: np.ndarray) -> bool:
        _, nn = self.static_obstacles.query(grid_pos)
        return self.l2(grid_pos, self.static_obstacles.data[nn]) > self.robot_radius

    '''
    Space-Time A*
    '''
    def plan(self, start: Tuple[int, int],
                   goal: Tuple[int, int],
                   dynamic_obstacles: Dict[int, Set[Tuple[int, int]]], # 动态障碍物，key为时间戳，value为位置
                   semi_dynamic_obstacles:Dict[int, Set[Tuple[int, int]]] = None,
                   max_iter:int = 500,
                   debug:bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:

        # Prepare dynamic obstacles
        self.dynamic_obstacles = dict((k, np.array(list(v))) for k, v in dynamic_obstacles.items())
        # 该函数将动态障碍物（dynamic_obstacles）字典的键值对转换为新的键值对的形式，key保持不变
        # 其中值保留为列表，并使用numpy库的array函数转换为数组形式。最终将转换后的字典赋值给dynamic_obstacles变量。
        # 目的是将values中的列表转换为numpy数组
        # Assume dynamic obstacles are agents with same radius, distance needs to be 2*radius 用于判断给定的位置和时间是否安全
        def safe_dynamic(grid_pos: np.ndarray, gride_angle: float, time: int) -> Tuple[bool, bool]:  # 1,是否与动态障碍物绝对安全 2,是否通过轨迹规划解决
            # nonlocal dynamic_obstacles # 非局部变量dynamic_obstacles
            for obstacle in self.dynamic_obstacles.setdefault(time, np.array([])):
                if self.l2(grid_pos, obstacle[0:2]) <= 2 * self.robot_radius:
                    if abs(gride_angle - obstacle[2]) == math.pi:
                        return False, False
                    elif gride_angle == math.inf:
                        return False, False
                    else:
                        return True, True

            return True, False
            # return all(self.l2(grid_pos, obstacle[0:2]) > 2 * self.robot_radius
            #            for obstacle in dynamic_obstacles.setdefault(time, np.array([])))

        # Prepare semi-dynamic obstacles, consider them static after specific timestamp
        if semi_dynamic_obstacles is None:
            semi_dynamic_obstacles = dict()
        else:
            semi_dynamic_obstacles = dict((k, np.array(list(v))) for k, v in semi_dynamic_obstacles.items())

        def safe_semi_dynamic(grid_pos: np.ndarray, time: int) -> bool:
            nonlocal semi_dynamic_obstacles
            for timestamp, obstacles in semi_dynamic_obstacles.items():
                flag = True
                if time >= timestamp:
                    flag = all(self.l2(grid_pos, obstacle) > 2 * self.robot_radius for obstacle in obstacles)
                if not flag:
                    return False
            return True


        start = self.grid.snap_to_grid(np.array(start))
        goal = self.grid.snap_to_grid(np.array(goal))

        # Initialize the start state
        s = State(start, math.inf, 0, 0, self.h(start, goal))

        open_set = [s]
        closed_set = set()

        # Keep track of parent nodes for reconstruction
        came_from = dict()
        came_from[s] = s

        iter_ = 0
        while open_set and iter_ < max_iter:
            iter_ += 1
            current_state = open_set[0]  # Smallest element in min-heap (heap=堆)
            if current_state.pos_equal_to(goal): # 判断当前位置是否与给定终点位置相等
                if debug:
                    print('STA*: Path found after {0} iterations'.format(iter_))
                return self.reconstruct_path(came_from, current_state, start=s)

            closed_set.add(heappop(open_set))
            epoch = current_state.time + 1
            for neighbour in self.neighbour_table.lookup(current_state.pos):
                if neighbour[0] == current_state.pos[0] and neighbour[1] == current_state.pos[1]:
                    angle = math.inf
                else:
                    angle = math.atan2(neighbour[1] - current_state.pos[1], neighbour[0] - current_state.pos[0])
                neighbour_state = State(neighbour, angle, epoch, current_state.g_score + 1, self.h(neighbour, goal, current_state, came_from[current_state]))
                # Check if visited
                if neighbour_state in closed_set:
                    continue

                # Avoid obstacles
                # 判断neighbour是否与静态障碍物和动态障碍物碰撞，以及是否与 semi-dynamic obstacles(半动态障碍物) 碰撞
                safe_dynamic_flag, solve_by_traj = safe_dynamic(neighbour, angle, epoch)

                if not safe_dynamic_flag:
                    print("collision dynamic obstacle, num of iter: ", iter_)
                # elif not self.safe_static(neighbour):
                #     print("collision static obstacle, num of iter: ", iter_)


                if not self.safe_static(neighbour) \
                   or not safe_dynamic_flag \
                   or not safe_semi_dynamic(neighbour, epoch):
                    continue

                if solve_by_traj:
                    neighbour_state.solve_by_traj = True

                # Add to open set
                if neighbour_state not in open_set:
                    came_from[neighbour_state] = current_state # state作为key，同时，state作为value
                    heappush(open_set, neighbour_state)

        if debug and iter_ == max_iter:
            print('STA*: Maximum iterations reached, no path found.')
        elif debug:
            print('STA*: Open set is empty, no path found.')
        self.plot_temp_result(open_set, closed_set, start, goal)
        # 显示open_set
        return np.array([])

    def plot_temp_result(self, open_set: List[State], closed_set: Set[State], start, goal):
        open_set_pos = []
        closed_set_pos = []
        for state in open_set:
            open_set_pos.append(state.pos)
        for state in closed_set:
            closed_set_pos.append(state.pos)
        open_set_pos = np.array(open_set_pos)
        closed_set_pos = np.array(closed_set_pos)
        plt.scatter(open_set_pos[:, 0], open_set_pos[:, 1], c='r', marker='o')
        plt.scatter(closed_set_pos[:, 0], closed_set_pos[:, 1], c='b', marker='o')
        plt.scatter(start[0], start[1], c='g', marker='o')
        plt.scatter(goal[0], goal[1], c='g', marker='^')
        plt.show()


    def check_the_bound(self, current: State, max_itr = 10) -> Tuple[int, int]:
        up_bound = 0
        low_bound = 0
        for i in range(1, max_itr):
            up_time = current.time + i
            for obstacle in self.dynamic_obstacles.setdefault(up_time, np.array([])):
                if np.array_equal(current.pos, obstacle[0:2]):
                    up_bound = i
                    break
        for i in range(1, max_itr):
            low_time = current.time - i
            for obstacle in self.dynamic_obstacles.setdefault(low_time, np.array([])):
                if np.array_equal(current.pos, obstacle[0:2]):
                    low_bound = -1
                    break
        return up_bound, low_bound

    '''
    Reconstruct path from A* search result
    '''
    def reconstruct_path(self, came_from: Dict[State, State], current: State, start: State) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        vertex_constraints = []
        up_bound_list = []
        low_bound_list = []
        total_path = [current.pos]
        total_path_angle = [current.theta]
        up_bound, low_bound = self.check_the_bound(current)
        up_bound_list.append(up_bound)
        low_bound_list.append(low_bound)
        if current.solve_by_traj:
            vertex_constraints.append(1)
        else:
            vertex_constraints.append(0)

        while current in came_from.keys():
            current = came_from[current]
            self.check_the_bound(current)
            up_bound, low_bound = self.check_the_bound(current)
            up_bound_list.append(up_bound)
            low_bound_list.append(low_bound)
            if current.theta == math.inf:
                current.theta = total_path_angle[-1]
            if current.solve_by_traj:
                vertex_constraints.append(1)
            else:
                vertex_constraints.append(0)
            total_path.append(current.pos)
            total_path_angle.append(current.theta)
            if current == start:
                break
        # print("up_bound_list: ", up_bound_list)
        # print("low_bound_list: ", low_bound_list)
        return np.array(total_path[::-1]), np.array(total_path_angle[::-1]), np.array(vertex_constraints[::-1]), np.array(up_bound_list[::-1]), np.array(low_bound_list[::-1])