import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple, Dict, Callable, Set

def get_map_data():
    # Open the file in read mode
    # ‘with’ makes sure that the file is closed automatically once the program exits the block
    filename_1 = '/home/tony/MAPF/MAP/warehouse-10-20-10-2-2.map'
    filename_2 = '/home/tony/MAPF/MAP/warehouse-test.map'
    filename_3 = '/home/tony/MAPF/MAP/warehouse-gazebo.map'

    with open(filename_3, 'r') as file:
        map_data = file.read()

    # Split the map into lines
    # strip() removes the leading and trailing whitespaces
    # split() splits the string on the basis of whitespace
    map_lines = map_data.strip().split('\n')
    map_index = next(i for i, line in enumerate(map_lines) if 'map' in line)
    height_index = next(i for i, line in enumerate(map_lines) if 'height' in line)
    width_index = next(i for i, line in enumerate(map_lines) if 'width' in line)

    # Get the lines containing the height and width information
    height_line = map_lines[height_index]
    width_line = map_lines[width_index]

    # Split the lines into words and get the second word (the number)
    height = int(height_line.split()[1])
    width = int(width_line.split()[1])
    # f-string
    # print(f"Height: {height}, Width: {width}")

    map_lines = map_lines[map_index+1:]

    # Extract obstacle coordinates
    obstacle_coordinates = []
    for row_idx, row in enumerate(map_lines):
        for col_idx, char in enumerate(row):
            if char == 'T':
                obstacle_coordinates.append((col_idx, row_idx))
    return height, width, obstacle_coordinates

def main():

    starts = [(17, 6), (3, 7), (8, 2)]
    goals = [(2, 16), (17, 12), (13, 17)]
    cmap = plt.get_cmap("viridis")
    colors = [cmap(i) for i in np.linspace(0, 1, len(starts))]

    # 依据height、width和obstacle_coordinates绘制地图
    height, width, obstacle_coordinates = get_map_data()
    print(type(obstacle_coordinates))
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)
    ax.set_xlim(-0.5, width-0.5)
    ax.set_ylim(-0.5, height-0.5)
    ax.set_xticks(np.arange(0.5, width, 1))
    ax.set_yticks(np.arange(0.5, height, 1))
    total_cells = height * width
    obstacle_cells = len(obstacle_coordinates)
    obstacle_ratio = obstacle_cells / total_cells
    print('height:', height)
    print('width:', width)
    # 计算出障碍物占地的比例
    print(f"Obstacle ratio: {obstacle_ratio}")

    # 绘制起点，用圆圈表示，颜色为红色
    for i in range(len(starts)):
        ax.plot(starts[i][0], starts[i][1], color=colors[i], marker='o', markersize=8)
        ax.text(starts[i][0], starts[i][1], f'Start {i + 1}', ha='right')
        ax.plot(goals[i][0], goals[i][1], color=colors[i], marker='^', markersize=8)
        if i == 0:
            ax.text(goals[i][0], goals[i][1], f'Goal {i + 1}', ha='left')
        else:
            ax.text(goals[i][0], goals[i][1], f'Goal {i + 1}', ha='right')

    ax.grid(True)
    for coordinate in obstacle_coordinates:
        ax.add_patch(plt.Rectangle((coordinate[0]-0.5, coordinate[1]-0.5), 1, 1, color='black'))

    plt.xlabel('X(m)')
    plt.ylabel('Y(M)')
    plt.show()

if __name__ == '__main__':
    main()