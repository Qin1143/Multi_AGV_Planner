#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com

An implementation of multi-agent path finding using conflict-based search
[Sharon et al., 2015]
'''
from typing import List, Tuple, Dict, Callable, Set
import multiprocessing as mp
from heapq import heappush, heappop
from itertools import combinations
from copy import deepcopy
import numpy as np

# The low level planner for CBS is the Space-Time A* planner
# https://github.com/GavinPHR/Space-Time-AStar
from Agv_planner.STAstar.planner import Planner as STPlanner
import time
from .constraint_tree import CTNode
from .constraints import Constraints
from .agent import Agent
from .assigner import *


class Planner:

    def __init__(self, grid_size: int,
                 robot_radius: float,
                 static_obstacles: List[Tuple[int, int]]):

        self.robot_radius = robot_radius
        self.st_planner = STPlanner(grid_size, robot_radius, static_obstacles)

    '''
    You can use your own assignment function, the default algorithm greedily assigns
    the closest goal to each start.
    '''

    def plan(self, starts: List[Tuple[int, int]],
             goals: List[Tuple[int, int]],
             assign: Callable = no_assign, # 是否使用任务分配算法
             max_iter: int = 200,
             low_level_max_iter: int = 100,
             max_process: int = 10,
             debug: bool = False) -> np.ndarray:

        self.low_level_max_iter = low_level_max_iter
        self.debug = debug

        # Do goal assignment
        self.agents = assign(starts, goals)
        self.calculate_time = 0

        constraints = Constraints()

        # Compute path for each agent using low level planner
        solution = dict((agent, self.calculate_path(agent, constraints, None)) for agent in self.agents)

        open = []
        if all(len(path) != 0 for path in solution.values()):
            '''
            all函数用于判断一个可迭代对象中的所有元素是否都为True，如果是，则返回True，否则返回False。
            在上述代码中，all函数用于判断字典solution中所有值的长度是否都不为0。
            如果所有值的长度都不为0，则返回True，如果存在一个值的长度为0，则返回False。
            '''
            # Make root node
            node = CTNode(constraints, solution)  # 创建一个CTNode对象，每个Node包含constraints，solution和cost三个属性
            # Min heap for quick extraction
            open.append(node)

        manager = mp.Manager()  # 创建一个Manager对象，用于管理共享对象。Manager对象用于在多进程环境中创建和管理跨进程共享的数据对象。
        iter_ = 0
        start_time = time.time()
        while open and iter_ < max_iter:  # 循环直到开放列表为空或者迭代次数超过最大迭代次数
            iter_ += 1  # 迭代次数加1
            results = manager.list([])  # 创建一个空列表用于存储结果
            processes = []  # 创建一个空列表用于存储进程

            # 最大进程数默认为10
            for _ in range(max_process if len(open) > max_process else len(open)):  # 根据最大进程数或者开放列表中的元素数量进行循环
                p = mp.Process(target=self.search_node,
                               args=[heappop(open), results])  # 创建一个进程，进程目标是搜索节点函数，并传入开放列表中的一个节点和结果列表作为参数
                                                               # mp.Process...在python的多进程模块multiprocessing中创建一个新的进程
                                                               # target参数用于指定进程执行的函数， args参数用于指定进程执行函数的参数
                                                               # heappop函数用于从开放列表中弹出最小的元素
                processes.append(p)  # 将进程添加到进程列表中
                p.start()  # 启动进程

            for p in processes:  # 遍历进程列表
                p.join()  # 等待进程结束

            for result in results:  # 遍历结果列表
                if len(result) == 1:  # 如果结果长度为1
                    if debug:  # 如果调试模式
                        print('CBS_MAPF: Paths found after about {0} iterations'.format(4 * iter_))  # 打印路径找到的消息
                        end_time = time.time()
                        self.calculate_time = end_time - start_time
                    return result[0]  # 返回结果中的第一个节点
                if result[0]:  # 如果结果的第一个节点存在
                    heappush(open, result[0])  # 将结果的第一个节点推入开放列表
                if result[1]:  # 如果结果的第二个节点存在
                    heappush(open, result[1])  # 将结果的第二个节点推入开放列表
        if debug:
            print('CBS-MAPF: Open set is empty, no paths found.')
        return np.array([])

    '''
    Abstracted away the cbs search for multiprocessing.
    The parameters open and results MUST BE of type ListProxy to ensure synchronization.
    '''

    def search_node(self, best: CTNode, results):
        agent_i, agent_j, time_of_conflict = self.validate_paths(self.agents, best)

        # If there is not conflict, validate_paths returns (None, None, -1) 无碰撞处理
        if agent_i is None:
            results.append((self.reformat(self.agents, best.solution),))  # 如果agent_i无碰撞，代表CTNode无碰撞，则将best.solution添加到results列表中
            return
        # Calculate new constraints
        if isinstance(time_of_conflict, int):  # 检查time_of_conflict是否是整数类型，如果是，则说明该冲突是顶点冲突
            agent_i_constraint = self.vertex_constraints(best, agent_i, agent_j, time_of_conflict)
            agent_j_constraint = self.vertex_constraints(best, agent_j, agent_i, time_of_conflict)
        else:  # 如果不是，则说明该冲突是边冲突
            agent_i_constraint = self.edge_constraints(best, agent_i, agent_j, time_of_conflict)
            agent_j_constraint = self.edge_constraints(best, agent_j, agent_i, time_of_conflict)

        # Calculate new paths
        agent_i_path = self.calculate_path(agent_i,
                                           agent_i_constraint,
                                           self.calculate_goal_times(best, agent_i, self.agents))
        agent_j_path = self.calculate_path(agent_j,
                                           agent_j_constraint,
                                           self.calculate_goal_times(best, agent_j, self.agents))

        # Replace old paths with new ones in solution
        solution_i = best.solution
        solution_j = deepcopy(best.solution)
        solution_i[agent_i] = agent_i_path
        solution_j[agent_j] = agent_j_path

        node_i = None
        if all(len(path) != 0 for path in solution_i.values()):
            node_i = CTNode(agent_i_constraint, solution_i)

        node_j = None
        if all(len(path) != 0 for path in solution_j.values()):
            node_j = CTNode(agent_j_constraint, solution_j)

        results.append((node_i, node_j))

    '''
    Pair of agent, point of conflict
    '''

    def validate_paths(self, agents, node: CTNode):
        """
        验证路径是否发生碰撞

        参数：
        agents(list[Agent]): 所有参与验证的Agent对象的列表
        node(CTNode): 需要验证的节点对象

        返回：
        tuple: 包含两个Agent对象和他们的冲突时间，如果没有发生碰撞则返回(None, None, -1)
        """

        # 对所有Agent对进行遍历，检查碰撞
        for agent_i, agent_j in combinations(agents, 2):  # 生成agents列表中所有两两组合的元素。
            # 获取两个Agent的安全距离
            time_of_conflict = self.safe_distance(node.solution, agent_i, agent_j)
            # 如果没有冲突，则time_of_conflict=-1
            if time_of_conflict == -1:
                continue
            # 返回冲突的两个Agent对象和他们的冲突时间
            return agent_i, agent_j, time_of_conflict

        # 如果没有发生碰撞，则返回(None, None, -1)
        return None, None, -1

    def safe_distance(self, solution: Dict[Agent, np.ndarray], agent_i: Agent, agent_j: Agent):
        """
        计算两个机器人之间的安全距离指数。

        参数：
        solution: dict，包含每个机器人当前位置的字典
        agent_i: Agent，第一个机器人
        agent_j: Agent，第二个机器人

        返回：
        int，安全距离指数。如果距离大于两倍机器人半径，则返回-1。
        """
        last_point = None

        for idx, (point_i, point_j) in enumerate(zip(solution[agent_i], solution[agent_j])):

            if self.dist(point_i, point_j) <= 2 * self.robot_radius:  # 如果碰撞
                return idx  # 顶点冲突返回冲突idx

            if idx != 0:
                if self.dist(last_point[0], point_j) <= 2 * self.robot_radius and self.dist(last_point[1], point_i) <= 2 * self.robot_radius:  # 如果边碰撞，两个机器人互换位置
                    return idx, idx - 1  # 边冲突返回冲突idx和idx-1，其实idx-1是需要的

            last_point = (point_i, point_j)

        return -1

    @staticmethod
    def dist(point1: np.ndarray, point2: np.ndarray) -> int:
        return int(np.linalg.norm(point1 - point2, 2))  # L2 norm 计算两点之间的距离

    def edge_constraints(self, node: CTNode,
                              agent_1: Agent,
                              agent_2: Agent,
                              time_of_conflict: Tuple[int, int]) -> Constraints:
        print('edge_constraints')
        agent_1_path = node.solution[agent_1]
        agent_2_path = node.solution[agent_2]
        obstacle1 = [agent_1_path[time_of_conflict[0]]]
        obstacle2 = [agent_2_path[time_of_conflict[0]]]
        times = [time_of_conflict[0]]
        print("T:", times, "obs1:", obstacle1, "obs2:", obstacle2)
        while True:
            t = times[-1] + 1
            if self.dist(agent_1_path[t], agent_1_path[t-3]) <= 2:  # agent1转向
                return node.constraints.fork_edge(agent_1, obstacle1, times)
            elif self.dist(agent_2_path[t], agent_2_path[t-3]) <= 2:  # agent2转向
                return node.constraints.fork_edge(agent_2, obstacle2, times)
            else:
                obstacle1.append(agent_1_path[t])
                obstacle2.append(agent_2_path[t])
                times.append(t)
                # break
        # contrained_path = node.solution[constrained_agent]  # 受限路径
        # unchanged_path = node.solution[unchanged_agent]  # 不受限路径
        # obstacle1 = unchanged_path[time_of_conflict[0]].tolist()
        # obstacle2 = unchanged_path[time_of_conflict[1]].tolist()
        # pivot = [obstacle1, obstacle2]
        # print("pivot:", pivot, "obs1:", obstacle1, "obs2:", obstacle2)  # pivot: [[25, 32], [25, 33]] obs1: [25, 32] obs2: [25, 33]
        # conflict_end_time = time_of_conflict[1]
        # return node.constraints.fork1(constrained_agent, pivot, time_of_conflict[0], conflict_end_time)

    def vertex_constraints(self, node: CTNode,
                              constrained_agent: Agent,
                              unchanged_agent: Agent,
                              time_of_conflict: int) -> Constraints:
        print('vertex_constraints')
        contrained_path = node.solution[constrained_agent]  # 受限路径
        unchanged_path = node.solution[unchanged_agent]  # 不受限路径

        pivot = unchanged_path[time_of_conflict] # 将pivot对象转换为一个具有相同数据但以嵌套列表形式表示的Python列表
        conflict_end_time = time_of_conflict
        try:  # try用于尝试执行一段可能会出现错误的代码，并处理可能发生的异常
            while self.dist(contrained_path[conflict_end_time], pivot) < 2 * self.robot_radius:
                conflict_end_time += 1  # 计算冲突结束的时间
        except IndexError:  # 索引错误
            pass  # 忽略错误
        return node.constraints.fork(constrained_agent, tuple(pivot.tolist()), time_of_conflict, conflict_end_time)

    def calculate_goal_times(self, node: CTNode, agent: Agent, agents: List[Agent]):
        solution = node.solution
        goal_times = dict()
        for other_agent in agents:
            if other_agent == agent:
                continue
            time = len(solution[other_agent]) - 1
            goal_times.setdefault(time, set()).add(tuple(solution[other_agent][time]))
        return goal_times

    '''
    Calculate the paths for all agents with space-time constraints
    '''

    def calculate_path(self, agent: Agent,
                       constraints: Constraints,
                       goal_times: Dict[int, Set[Tuple[int, int]]]) -> np.ndarray:
        return self.st_planner.plan(agent.start,
                                    agent.goal,
                                    constraints.setdefault(agent, dict()),
                                    semi_dynamic_obstacles=goal_times,
                                    max_iter=self.low_level_max_iter,
                                    debug=self.debug)

    '''
    Reformat the solution to a numpy array
    '''

    @staticmethod
    def reformat(agents: List[Agent], solution: Dict[Agent, np.ndarray]):
        solution = Planner.pad(solution)
        reformatted_solution = []
        for agent in agents:
            reformatted_solution.append(solution[agent])
        return np.array(reformatted_solution)

    '''
    Pad paths to equal length, inefficient but well..
    '''

    @staticmethod
    def pad(solution: Dict[Agent, np.ndarray]):
        max_ = max(len(path) for path in solution.values())
        for agent, path in solution.items():
            if len(path) == max_:
                continue
            padded = np.concatenate([path, np.array(list([path[-1]]) * (max_ - len(path)))])
            solution[agent] = padded
        return solution
