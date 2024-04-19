"""
Plot tools 2D and 3D
@author: Ziqi Qin
"""
import math
import os
import sys
# import matplotlib
# matplotlib.use('Agg')  # 使用PNG后端
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as patches
from matplotlib import colors
from mpl_toolkits.mplot3d import Axes3D
from create_map import get_map_data
import numpy as np

class Plotting:
    def __init__(self, starts, goals, mission_num, height, width, obs, paths: dict, paths_angle: dict,corners_index: dict,
                 DP_paths_all: dict, QP_paths_all: dict, S: dict, up_dp_bound: dict, low_dp_bound: dict,
                 dp_vertex_constraints: dict, final_x: dict, final_y: dict):
        self.starts, self.goals = starts, goals
        self.height, self.width, self.obs = height, width, obs
        self.paths = paths
        self.paths_angle = paths_angle
        self.corners_index = corners_index
        cmap = plt.get_cmap("viridis")
        self.mission_num = mission_num
        self.DP_paths_all = DP_paths_all
        self.QP_paths_all = QP_paths_all
        self.S = S
        self.up_dp_bound = up_dp_bound
        self.low_dp_bound = low_dp_bound
        self.dp_vertex_constraints = dp_vertex_constraints
        self.final_x = final_x
        self.final_y = final_y
        self.colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        # self.multi_plot_2D_static(self.paths)
        # self.multi_plot_2D_dynamic()
        self.multi_plot_2D_semi_dynamic()
        # self.plot_show_DP_QP(self.DP_paths_all, self.QP_paths_all)
        # self.plot_xyt_result()

    def multi_plot_2D_semi_dynamic(self):
        max_time = 0
        for i in range(self.mission_num):
            if len(self.final_x[i]) / 100 > max_time:
                max_time = len(self.final_x[i]) / 100

        final_time = 0
        for i in range(self.mission_num):
            end_time = len(self.final_x[i]) * 0.01
            if end_time > final_time:
                final_time = end_time
        time_points = np.linspace(0, final_time, 9)
        earlier_time = 500

        for T in time_points[1:]:
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.set_aspect('equal')
            ax.set_xlabel('X(m)')
            ax.set_ylabel('Y(m)')
            ax.set_xlim(-0.5, self.width - 0.5)
            ax.set_ylim(-0.5, self.height - 0.5)
            ax.set_xticks(np.arange(0.5, self.width, 1))
            ax.set_yticks(np.arange(0.5, self.height, 1))
            for obs in self.obs:
                ax.add_patch(plt.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
            ax.grid(True)

            for i in range(self.mission_num):
                ax.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8, alpha=0.5)
                ax.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8, alpha=0.5)
                # ax.annotate(str(i + 1), (self.starts[i][0], self.starts[i][1]))

                # Convert time to index
                index_T = min(len(self.final_x[i]) - 1,
                              int(T * 100))  # Assuming final_x and final_y are sampled at 100Hz
                if index_T == len(self.final_x[i]) - 1:
                    index_T_earlier = min(int(T * 100) - earlier_time, index_T)
                else:
                    index_T_earlier = max(0, index_T - earlier_time)  # 3 seconds earlier

                # Get the positions at T and T-5 seconds
                positions_at_T = self.final_x[i][index_T], self.final_y[i][index_T]
                positions_last_5_seconds = self.final_x[i][index_T_earlier:index_T], self.final_y[i][
                                                                                     index_T_earlier:index_T]

                # Plot the positions at T as a dot
                ax.plot(*positions_at_T, color=self.colors[i], marker='o', markersize=8, alpha=1)
                ax.annotate(str(i + 1), (self.final_x[i][index_T], self.final_y[i][index_T]))

                # Plot the positions in the last 5 seconds as a line
                ax.plot(*positions_last_5_seconds, color=self.colors[i], linewidth=2, alpha=0.5)

            name = f'{self.mission_num}Agents, T={T}'
            # ax.set_title(name)
            plt.savefig(f'/home/tony/MAPF/Figures/{self.mission_num}Agents_{T}s.pdf')
            plt.savefig(f'/home/tony/MAPF/Figures/{self.mission_num}Agents_{T}s.png')
            plt.show()
    def plot_xyt_result(self):
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        # 在xy平面内绘制障碍物
        for x, y in zip(obs_x, obs_y):
            # 创建一个立方体
            z = 0  # z坐标
            dx = dy = dz = 1  # 长度
            cube = [(x - dx / 2, y - dy / 2, z), (x + dx / 2, y - dy / 2, z), (x + dx / 2, y + dy / 2, z),
                    (x - dx / 2, y + dy / 2, z)]  # 底面的四个顶点
            cube = [cube]  # 转换格式
            ax.add_collection3d(Poly3DCollection(cube, facecolors='black', linewidths=0.5, edgecolors='black', alpha=0.2))

        for i in range(self.mission_num):
            x = self.final_x[i]
            y = self.final_y[i]
            zeros = np.zeros(len(x))
            t2 = np.arange(0, len(x) * 0.01, 0.01)
            ax.plot(x, y, zeros, linewidth='3', color=self.colors[i], alpha=0.7)
            ax.plot(x, zeros, t2, linewidth='3', color=self.colors[i], alpha=0.2)
            ax.plot(zeros, y, t2, linewidth='3', color=self.colors[i], alpha=0.2)
            ax.plot(x, y, t2, linewidth='3', color=self.colors[i], alpha=1, label=f'Robot {i+1}')
            ax.plot(self.starts[i][0], self.starts[i][1], 0,color=self.colors[i], marker='o', markersize=8)
            ax.plot(self.goals[i][0], self.goals[i][1], 0, color=self.colors[i], marker='^', markersize=8)
        #设置坐标轴范围
        ax.set_xlim(-0.5, self.width-0.5)
        ax.set_ylim(-0.5, self.height-0.5)
        max_t = max(len(v) for v in self.final_x.values()) * 0.01
        ax.set_zlim(0, max_t)
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m)')
        ax.set_zlabel('T(s)')
        # 设置视角
        ax.view_init(30, 45)

        plt.legend()
        plt.show()

    def multi_plot_2D_dynamic(self):
        # self.plot_grid()
        self.plot_dynamic_paths()
        # plt.show()

    def plot_dynamic_paths(self):
        """
        创建一个动画，展示机器人的运动轨迹final_x和final_y
        """
        fig, ax = plt.subplots(figsize=(8, 6))
        # ax.set_xlim(0-self.width/10, self.width+self.width/10)
        # ax.set_ylim(0-self.height/10, self.height+self.height/10)
        ax.set_aspect('equal')
        name = f'Multi_AGV_Planner_{self.mission_num}Agents'
        ax.set_title(name)
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m)')
        max_time = 0
        for i in range(self.mission_num):
            if len(self.final_x[i])/100 > max_time:
                max_time = len(self.final_x[i])/100
            ax.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8, alpha=0.5)
            ax.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8, alpha=0.5)
            ax.annotate(str(i+1), (self.starts[i][0], self.starts[i][1]))
            # path_x = [point[0] for point in self.paths[i]]
            # path_y = [point[1] for point in self.paths[i]]
            # corners_x = [path_x[j] for j in self.corners_index[i]]
            # corners_y = [path_y[j] for j in self.corners_index[i]]
            # ax.plot(path_x, path_y, linewidth='3', color=self.colors[i], alpha=0.5)
            # ax.plot(corners_x, corners_y, 'o', color=self.colors[i], alpha=1)
        ax.set_xlim(-0.5, self.width - 0.5)
        ax.set_ylim(-0.5, self.height - 0.5)
        ax.set_xticks(np.arange(0.5, self.width, 1))
        ax.set_yticks(np.arange(0.5, self.height, 1))
        for obs in self.obs:
            ax.add_patch(plt.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
        ax.grid(True)
        # obs_x = [obs[0] for obs in self.obs]
        # obs_y = [obs[1] for obs in self.obs]



        # ax.plot(obs_x, obs_y, "sk")

        dots = []
        for i in range(self.mission_num):
            dot, = ax.plot([], [], color=self.colors[i], marker='o', markersize=6, alpha=1)
            dots.append(dot)

        def init():
            for dot in dots:
                dot.set_data([], [])
            return dots

        def animate(i):
            for j in range(self.mission_num):
                if i < len(self.final_x[j]) and i < len(self.final_y[j]):  # 添加这个条件检查
                    dots[j].set_data(self.final_x[j][i], self.final_y[j][i])
                else:
                    dots[j].set_data(self.final_x[j][-1], self.final_y[j][-1])
            return dots

        # lines = []
        # for i in range(self.mission_num):
        #     l, = ax.plot([], [], color=self.colors[i], linewidth=3, alpha=0.8)
        #     lines.append(l)
        #
        # def init():
        #     for line in lines:
        #         line.set_data([], [])
        #     return lines
        #
        # def animate(i):
        #     for j in range(self.mission_num):
        #         lines[j].set_data(self.final_x[j][:i], self.final_y[j][:i])
        #     return lines

        speed = 10
        frame_rate = 100  # 每秒的帧数
        frames = round(max_time * frame_rate)  # 总帧数
        interval = 1000 / (speed * frame_rate)  # 每帧之间的时间间隔，单位为毫秒
        ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=interval, blit=True) # interval=10表示每隔10ms更新一次，也是正常速度 interval=1表示十倍速
        filename_gif = f'/home/tony/MAPF/Figures/animation_{self.mission_num}agents.gif'
        filename_mp4 = f'/home/tony/MAPF/Figures/animation_{self.mission_num}agents_gazebo.mp4'
        # ani.save(filename_gif, writer='pillow')
        ani.save(filename_mp4, writer='ffmpeg')
        plt.show()


    def multi_plot_2D_static(self, paths):
        #仅显示path
        self.plot_grid()
        self.plot_static_paths(paths)

        #色阶显示
        # self.plot_2D_t()
        plt.show()

    def plot_grid(self):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        # plt.figure(figsize=(16, 12))
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        ax.set_xlim(-0.5, self.width - 0.5)
        ax.set_ylim(-0.5, self.height - 0.5)
        ax.set_xticks(np.arange(0.5, self.width, 1))
        ax.set_yticks(np.arange(0.5, self.height, 1))
        # plt.get_current_fig_manager().full_screen_toggle()
        for i in range(self.mission_num):
            ax.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8)
            ax.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8)
            # ax.annotate('Start ' + str(i+1), (self.starts[i][0], self.starts[i][1]))
            # ax.annotate('Goal ' + str(i+1), (self.goals[i][0], self.goals[i][1]))
            ax.text(self.starts[i][0], self.starts[i][1], f'Start {i + 1}', ha='right')
            if i == 0:
                ax.text(self.goals[i][0], self.goals[i][1], f'Goal {i + 1}', ha='left')
            else:
                ax.text(self.goals[i][0], self.goals[i][1], f'Goal {i + 1}', ha='right')
        # plt.plot(obs_x, obs_y, "sk", alpha=0.5)
        for obs in self.obs:
            ax.add_patch(plt.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
        ax.grid(True)
        plt.xlabel('X(m)')
        plt.ylabel('Y(M)')
        # name = f'Multi_AGV_Planner_{self.mission_num}Agents'
        # plt.title(name)
        plt.axis("equal")

    def plot_static_paths(self, paths):
        for i in range(self.mission_num):
            path_x = [point[0] for point in paths[i]]
            path_y = [point[1] for point in paths[i]]
            # corners_x = [path_x[j] for j in self.corners_index[i]]
            # corners_y = [path_y[j] for j in self.corners_index[i]]
            plt.plot(path_x, path_y, linewidth='3', color=self.colors[i], alpha=0.8)
            # plt.plot(corners_x, corners_y, 'o', color=self.colors[i], alpha=1)

            # for j in range(len(path_x) - 1):  # 每隔10个点绘制一个箭头
            #     if j % 10 == 0:  # 只在每隔10个点处绘制箭头
            #         dx = path_x[j + 1] - path_x[j]
            #         dy = path_y[j + 1] - path_y[j]
            #         plt.quiver(path_x[j], path_y[j], dx, dy, angles='xy', scale_units='xy', scale=1,
            #                    color=self.colors[i])

    def plot_2D_t(self):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        # plt.figure(figsize=(16, 12))
        plt.figure(figsize=(8, 6))
        # plt.get_current_fig_manager().full_screen_toggle()
        for i in range(self.mission_num):
            plt.plot(self.starts[i][0], self.starts[i][1], color=self.colors[i], marker='o', markersize=8)
            plt.plot(self.goals[i][0], self.goals[i][1], color=self.colors[i], marker='^', markersize=8)
            plt.annotate(str(i), (self.starts[i][0], self.starts[i][1]))
        plt.plot(obs_x, obs_y, "sk", alpha=0.5)

        # 创建一个ScalarMappable对象，用于将时间转换为颜色
        cmap = plt.get_cmap("viridis")
        norm = colors.Normalize(vmin=0, vmax=max(len(v) for v in self.final_x.values()) * 0.01)
        sm = ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        for i in range(self.mission_num):
            x = self.final_x[i]
            y = self.final_y[i]
            t = np.arange(0, len(x) * 0.01, 0.01)
            color_values = sm.to_rgba(t)

            plt.scatter(x, y, linewidths=3, edgecolors=color_values, color=color_values, alpha=0.8,
                        label=f'Agent_{i + 1}')
        plt.legend()
        plt.xlabel('X(m)')
        plt.ylabel('Y(M)')
        name = f'Multi_AGV_Planner_{self.mission_num}Agents'
        plt.title(name)
        plt.axis("equal")


    def plot_show_DP_QP(self, DP_paths_all, QP_paths_all):
        # 展示轨迹
        # cmap = plt.get_cmap("viridis")
        # colors = [cmap(i) for i in np.linspace(0, 1, self.mission_num)]
        # print(self.mission_num)
        Tunnel = dict()
        Tunnel[0] = [[0, 0],[2, 0], [2, 2], [0, 2]]
        Tunnel[1] = [[3, 2], [7, 2], [7, 6], [16, 15], [16, 17], [3, 17]]
        Tunnel[2] = [[17, 17], [20, 17], [20, 20], [17, 20]]
        for i in range(self.mission_num):
            # Tunnel[i] = [[DP_paths_all[i][0, 1], DP_paths_all[i][0, 0]], [DP_paths_all[i][-1, 1], DP_paths_all[i][0, 0]],
            #              [DP_paths_all[i][-1, 1], DP_paths_all[i][-1, 0]], [DP_paths_all[i][0, 1], DP_paths_all[i][-1, 0]]]
            if i == 2:
                polygon0 = Polygon(Tunnel[0], fill=True, facecolor='orange', edgecolor='orange', alpha=0.2)
                polygon1 = Polygon(Tunnel[1], fill=True, facecolor='orange', edgecolor='orange', alpha=0.2)
                polygon2 = Polygon(Tunnel[2], fill=True, facecolor='orange', edgecolor='orange', alpha=0.2)
            fig = plt.figure(figsize=(8, 6))
            ax = fig.add_subplot(111)
            if i == 2:
                ax.add_patch(polygon0)
                ax.add_patch(polygon1)
                ax.add_patch(polygon2)
                ax.text(6.5, 13.5, f'Feasible Tunnel')
            Tims = np.arange(len(self.paths[i]))
            plt.plot(Tims, self.S[i], label='ISTA*')#, colors[i])
            plt.plot(DP_paths_all[i][:, 1], DP_paths_all[i][:, 0], label='DP Speed Profile')#, colors[i])
            # plt.plot(QP_paths_all[i][:, 1], QP_paths_all[i][:, 0], label='QP Result')#, colors[i])
            # plt.plot(Tims, self.up_dp_bound[i] + self.S[i], 'r', label='Feasible Tunnel')
            # plt.plot(Tims, self.low_dp_bound[i] + self.S[i], 'r')
            obs_label = True
            for index, k in enumerate(self.dp_vertex_constraints[i]):
                if k == 1:
                    if obs_label is True:
                        rect1 = plt.Rectangle((Tims[index] - 0.5, self.S[i][index] - 0.5), 1, 1,
                                              color='red', alpha=0.2)
                        plt.gca().add_patch(rect1)
                        plt.plot(Tims[index], self.S[i][index], 'ro', markersize=5, label='Obstacles')
                        obs_label = False
                    else:
                        rect1 = plt.Rectangle((Tims[index] - 0.5, self.S[i][index] - 0.5), 1, 1,
                                              color='red', alpha=0.2)
                        plt.gca().add_patch(rect1)
                        plt.plot(Tims[index], self.S[i][index], 'ro', markersize=5)

            # 图例
            plt.legend()
            plt.xlabel('T(s)')
            plt.ylabel('S(m)')
            # plt.title('ST_Figure_mission_' + str(i+1))
            plt.show()