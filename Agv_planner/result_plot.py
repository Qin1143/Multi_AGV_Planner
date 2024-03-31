import matplotlib.pyplot as plt
import numpy as np

def plot_success_rate():
    # 数据
    robots = [10, 15, 20, 25, 30, 35, 40]
    percentages_ours = [97.00, 97.30, 93.00, 94.80, 92.90, 85.30, 72.20]
    percentages_CBS = [96.50, 90.10, 87.60, 83.75, 76.60, 67.85, 52.40]

    # 在每个数据点位置添加对应的百分比
    for i in range(len(robots)):
        plt.text(robots[i], percentages_ours[i], f'{percentages_ours[i]}%', ha='center')
        plt.text(robots[i], percentages_CBS[i], f'{percentages_CBS[i]}%', ha='center')

    # 绘制线图
    plt.plot(robots, percentages_ours, marker='s', linewidth=2, color='r', label='Ours')
    plt.plot(robots, percentages_CBS, marker='s', linewidth=2, color='b', label='CBS+QP')

    # 设置图表标题和坐标轴标签
    # plt.title('Success Rate')
    plt.xlabel('Number of Agents')
    plt.ylabel('Success Rate(%)')
    plt.legend()

    plt.xlim(10, 40)
    plt.ylim(0, 100)

    # 显示图表
    plt.show()

def plot_box():
    # 数据
    agent_10 = [7.7344, 7.4829, 44.217, 14.2144, 6.0264, 5.4956, 34.8377]
    agent_20 = [14.496, 53.9626, 18.9867, 133.2628, 43.094, 30.8261]
    agent_30 = [109.8199, 81.5182, 64.2342, 30.8723, 44.6006, 122.6457, 74.1338, 31.2668]
    agent_40 = [100.3744, 74.3695, 125.7926, 102.8364, 46.5429, 73.2609]
    data = [agent_10, agent_20, agent_30, agent_40]

    # 绘制盒图，设置showfliers参数为False
    plt.boxplot(data, showfliers=False, widths=0.5, patch_artist=True,
                boxprops={'facecolor': 'None', 'color': 'black', 'linewidth': 1},
                whiskerprops={'linewidth': 1},
                capprops={'linewidth': 1},
                medianprops={'linewidth': 1})

    # 计算每组数据的平均值
    means = [np.mean(dataset) for dataset in data]

    # 在图中显示平均值数据
    # for i, mean in enumerate(means, start=1):
    #     plt.text(i, mean, f'Mean: {mean:.2f}', ha='center', va='bottom')

    # 设置每组数据所对应的横轴数据
    plt.xticks([1, 2, 3, 4], ['10', '20', '30', '40'])

    # 设置图表标题和坐标轴标签
    # plt.title('Box Plot')
    plt.xlabel('Number of Agents')
    plt.ylabel('Calculation Time(s)')

    # 显示图表
    plt.show()

def plot_average_time():
    # 数据
    agent_10 = [7.7344, 7.4829, 15.217, 14.2144, 6.0264, 5.4956, 17.8377]
    agent_20 = [34.496, 23.9626, 18.9867, 23.2628, 26.094, 30.8261]
    agent_30 = [39.8199, 45.5182, 54.2342, 38.8723, 44.6006, 42.6457, 54.1338, 46.2668]
    agent_40 = [67.3744, 74.3695, 72.7926, 72.8364, 46.5429, 53.2609]
    data = [agent_10, agent_20, agent_30, agent_40]

    # 计算每组数据的平均值
    means = [np.mean(dataset) for dataset in data]

    CBS = [6.80, 19.84, 39.57, 73.62]

    # 在每个数据点位置添加对应的数值
    for i in range(len(means)):
        plt.text(i, means[i], f'{means[i]:.2f}', ha='center', va='center')
        plt.text(i, CBS[i], f'{CBS[i]:.2f}', ha='center', va='center')

    # 绘制折线图
    plt.plot(['10', '20', '30', '40'], means, marker='s', linewidth=2, color='r', label='Ours')
    plt.plot(['10', '20', '30', '40'], CBS, marker='s', linewidth=2, color='b', label='CBS+QP')


    # 设置图表标题和坐标轴标签
    plt.xlabel('Number of Agents')
    plt.ylabel('Average Calculation Time(s)')
    plt.legend()
    # 显示图表
    plt.show()

if __name__ == '__main__':
    # plot_success_rate()
    # plot_box()
    plot_average_time()