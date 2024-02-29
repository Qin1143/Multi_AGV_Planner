import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple, Dict, Callable, Set

def get_map_data():
    # Open the file in read mode
    # ‘with’ makes sure that the file is closed automatically once the program exits the block
    filename_1 = '/home/tony/MAPF/MAP/warehouse-10-20-10-2-2.map'
    filename_2 = '/home/tony/MAPF/MAP/warehouse-test.map'
    with open(filename_2, 'r') as file:
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
    # 依据height、width和obstacle_coordinates绘制地图
    height, width, obstacle_coordinates = get_map_data()
    print(type(obstacle_coordinates))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_xticks(range(width))
    ax.set_yticks(range(height))

    ax.grid(True)
    for coordinate in obstacle_coordinates:
        ax.add_patch(plt.Rectangle((coordinate[0], coordinate[1]), 1, 1, color='black'))
    plt.show()

if __name__ == '__main__':
    main()