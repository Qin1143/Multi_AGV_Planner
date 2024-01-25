#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import Dict, Tuple, Set, List
from copy import deepcopy

from .agent import Agent

'''
Emulated dictionary of dictionaries
'''
class Constraints:

    def __init__(self):
        #                                      time,         obstacles
        self.agent_constraints: Dict[Agent: Dict[int, Set[Tuple[int, int]]]] = dict()

    '''
    Deepcopy self with additional constraints
    '''
    def fork(self, agent: Agent, obstacle: Tuple[int, int], start: int, end: int) -> 'Constraints':
        '''
        :param agent:
        :param obstacle: 冲突点，当作障碍物来处理
        :param start: conflict_start_time
        :param end: conflict_end_time
        :return:
        '''
        # 深拷贝 agent_constraints
        agent_constraints_copy = deepcopy(self.agent_constraints)
        # 对每个时间步，将agent不能经过的障碍物添加到agent_constraints_copy中
        for time in range(start, end):
            agent_constraints_copy.setdefault(agent, dict()).setdefault(time, set()).add(obstacle)
            # 这个函数将给定的agent作为键，如果约束字典中不存在agent，则在约束字典中创建一个空字典作为agent的默认值。
            # 然后，将给定的时间作为键，如果该时间不存在于agent的字典中，则在该字典中创建一个空集合作为时间的默认值。
            # 最后，将给定的障碍物添加到时间的集合中。
        # 创建新的Constraints对象
        new_constraints = Constraints()
        # 将生成的agent_constraints_copy赋值给new_constraints的agent_constraints属性
        new_constraints.agent_constraints = agent_constraints_copy
        # 返回新的Constraints对象
        return new_constraints

    def fork_edge(self, agent: Agent, obstacle: List, times:List) -> 'Constraints':
        agent_constraints_copy = deepcopy(self.agent_constraints)
        for i in range(len(times)):
            obstacle_tuple = tuple(obstacle[i])
            agent_constraints_copy.setdefault(agent, dict()).setdefault(times[i], set()).add(obstacle_tuple)
        new_constraints = Constraints()
        new_constraints.agent_constraints = agent_constraints_copy
        return new_constraints

    def setdefault(self, key, default):
        return self.agent_constraints.setdefault(key, default)

    def __getitem__(self, agent):
        return self.agent_constraints[agent]

    def __iter__(self):
        for key in self.agent_constraints:
            yield key

    def __str__(self):
        return str(self.agent_constraints)


