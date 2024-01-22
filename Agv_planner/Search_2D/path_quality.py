"""
Path Quality
@author: ziqi qin
"""

import math

class PathQuality:
    def __init__(self, path):
        self.path = path
        self.lin_distance = self.linear_distance()
        self.distanc = self.path_distance()
        self.num_turn = self.path_turn()

    def linear_distance(self):
        start_point = self.path[0]
        goal_point = self.path[-1]
        linear_distance = math.dist(start_point, goal_point)
        return linear_distance

    def path_distance(self):
        path_distance = 0
        for point in self.path:
            if point == self.path[0]:
                last_point = point
                continue
            else:
                path_distance += math.dist(point, last_point)
                last_point = point

        return path_distance

    def path_turn(self):
        path_turn = 0
        for i in range(1, len(self.path) - 2):
            point1_x, point1_y = self.path[i - 1]
            point2_x, point2_y = self.path[i]
            point3_x, point3_y = self.path[i + 1]
            if point1_x == point2_x == point3_x or point1_y == point2_y == point3_y:
                continue
            else:
                path_turn += 1
        return path_turn

# path = [(5, 5), (6, 5), (7, 5), (8, 5), (9, 5), (10, 5), (10, 6), (11, 6), (12, 6), (13, 6), (14, 6), (15, 6), (16, 6), (17, 6), (17, 7), (18, 7), (19, 7), (20, 7), (21, 7), (22, 7), (23, 7), (23, 8), (24, 8), (25, 8), (26, 8), (26, 9), (26, 10), (26, 11), (26, 12), (26, 13), (26, 14), (26, 15), (26, 16), (26, 17), (26, 18), (26, 19), (26, 20), (26, 21), (27, 21), (28, 21), (29, 21), (30, 21), (31, 21), (31, 22), (31, 23), (32, 23), (33, 23), (34, 23), (34, 24), (34, 25), (35, 25), (36, 25), (37, 25), (38, 25), (39, 25), (40, 25), (41, 25), (42, 25), (43, 25), (44, 25), (45, 25)]
# pq = PathQuality(path)
# print(pq.lin_distance, pq.distanc, pq.num_turn)