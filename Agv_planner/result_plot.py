import matplotlib.pyplot as plt

def plot_success_rate():
    # 数据
    robots = [10, 15, 20, 25, 30, 35, 40]
    percentages = [97.00, 97.30, 93.00, 94.80, 94.90, 92.00, 81.20]

    # 在每个数据点位置添加对应的百分比
    for i in range(len(robots)):
        plt.text(robots[i], percentages[i], f'{percentages[i]}%', ha='center')

    # 绘制线图
    plt.plot(robots, percentages, marker='s', linewidth=2, color='r')

    # 设置图表标题和坐标轴标签
    # plt.title('Success Rate')
    plt.xlabel('Number of Agents')
    plt.ylabel('Success Rate(%)')

    plt.xlim(10, 40)
    plt.ylim(0, 100)

    # 显示图表
    plt.show()
if __name__ == '__main__':
    plot_success_rate()