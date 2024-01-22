from Search_2D import plotting, env
import random
from Search_2D.Astar import AStar
from Search_2D.AGV_JPS_Astar import AgvAStar, AgvJpsAstar
from path_quality import PathQuality as pq
from tqdm import tqdm
import pandas as pd
import openpyxl
from create_map import get_map_data

def main():
    Env = env.Env(use_benchmark=True)
    height = Env.y_range
    width = Env.x_range
    columns = ['Calculation Time', 'Linear Distance', 'Euclidean Distance', 'Turn Count', 'Num Close', 'Start Point',
               'End Point']
    analize_col = ['Algorithm', 'Average Calculation Time', 'Average Euclidean Distance', 'Average Turn Count', 'Average Size Close']
    analize_row = ['AStar_E', 'AStar_M', 'Agv_AStar', 'Agv_Jps_Astar']
    Analize_data = pd.DataFrame(columns=analize_col)
    Analize_data['Algorithm'] = analize_row

    AStar_e_results_df = pd.DataFrame(columns=columns)
    AStar_m_results_df = pd.DataFrame(columns=columns)
    Agv_AStar_results_df = pd.DataFrame(columns=columns)
    Agv_Jps_Astar_results_df = pd.DataFrame(columns=columns)
    num_iterations = 50
    # 在地图的左右两侧分别设置50组随机的起点和终点，并储存在excel文件中
    Starts = []
    Goals = []

    for i in range(num_iterations):
        start_x = random.randint(1, 25)
        start_y = random.randint(1, height-2)
        goal_x = random.randint(width-1-25, width-2)
        goal_y = random.randint(1, height-2)
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        Starts.append(start)
        Goals.append(goal)

    for j in tqdm(range(num_iterations)):
        astar_e = AStar(Starts[j], Goals[j], "euclidean")
        aster_m = AStar(Starts[j], Goals[j], "manhattan")
        agv_astar = AgvAStar(Starts[j], Goals[j], "manhattan")
        agv_jps_astar = AgvJpsAstar(Starts[j], Goals[j], "manhattan")

        path, visited, time = astar_e.searching()
        # print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
        #       pq(path).num_turn, 'Num_close:', len(visited), '起点：', start, '终点：', goal)
        # 将每次的计算结果储存在新建的excel文件中
        result_row = pd.DataFrame({
            'Calculation Time': [time],
            'Linear Distance': [pq(path).lin_distance],
            'Euclidean Distance': [pq(path).distanc],
            'Turn Count': [pq(path).num_turn],
            'Num Close': [len(visited)],
            'Start Point': [Starts[j]],
            'End Point': [Goals[j]]
        })
        AStar_e_results_df = AStar_e_results_df.append(result_row, ignore_index=True)

        path, visited, time = aster_m.searching()
        # print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
        #       pq(path).num_turn, 'Num_close:', len(visited), '起点：', start, '终点：', goal)
        # 将每次的计算结果储存在新建的excel文件中
        result_row = pd.DataFrame({
            'Calculation Time': [time],
            'Linear Distance': [pq(path).lin_distance],
            'Euclidean Distance': [pq(path).distanc],
            'Turn Count': [pq(path).num_turn],
            'Num Close': [len(visited)],
            'Start Point': [Starts[j]],
            'End Point': [Goals[j]]
        })
        AStar_m_results_df = AStar_m_results_df.append(result_row, ignore_index=True)

        path, visited, time = agv_astar.searching()
        # print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
        #       pq(path).num_turn, 'Num_close:', len(visited), '起点：', start, '终点：', goal)
        # 将每次的计算结果储存在新建的excel文件中
        result_row = pd.DataFrame({
            'Calculation Time': [time],
            'Linear Distance': [pq(path).lin_distance],
            'Euclidean Distance': [pq(path).distanc],
            'Turn Count': [pq(path).num_turn],
            'Num Close': [len(visited)],
            'Start Point': [Starts[j]],
            'End Point': [Goals[j]]
        })
        Agv_AStar_results_df = Agv_AStar_results_df.append(result_row, ignore_index=True)

        path, visited, time = agv_jps_astar.searching()
        # print('计算时间：', time, '直线距离：', pq(path).lin_distance, '欧式距离：', pq(path).distanc, '转角次数：',
        #       pq(path).num_turn, 'Num_close:', len(visited), '起点：', start, '终点：', goal)
        # 将每次的计算结果储存在新建的excel文件中
        result_row = pd.DataFrame({
            'Calculation Time': [time],
            'Linear Distance': [pq(path).lin_distance],
            'Euclidean Distance': [pq(path).distanc],
            'Turn Count': [pq(path).num_turn],
            'Num Close': [len(visited)],
            'Start Point': [Starts[j]],
            'End Point': [Goals[j]]
        })
        Agv_Jps_Astar_results_df = Agv_Jps_Astar_results_df.append(result_row, ignore_index=True)

    Analize_data['Average Calculation Time'] = [AStar_e_results_df['Calculation Time'].mean(), AStar_m_results_df['Calculation Time'].mean(), Agv_AStar_results_df['Calculation Time'].mean(), Agv_Jps_Astar_results_df['Calculation Time'].mean()]
    Analize_data['Average Euclidean Distance'] = [AStar_e_results_df['Euclidean Distance'].mean(), AStar_m_results_df['Euclidean Distance'].mean(), Agv_AStar_results_df['Euclidean Distance'].mean(), Agv_Jps_Astar_results_df['Euclidean Distance'].mean()]
    Analize_data['Average Turn Count'] = [AStar_e_results_df['Turn Count'].mean(), AStar_m_results_df['Turn Count'].mean(), Agv_AStar_results_df['Turn Count'].mean(), Agv_Jps_Astar_results_df['Turn Count'].mean()]
    Analize_data['Average Size Close'] = [AStar_e_results_df['Num Close'].mean(), AStar_m_results_df['Num Close'].mean(), Agv_AStar_results_df['Num Close'].mean(), Agv_Jps_Astar_results_df['Num Close'].mean()]
    Analize_data.to_excel('Analize_data.xlsx', index=True)

    # AStar_e_results_df.to_excel('AStar_e_results.xlsx', index=True)
    # AStar_m_results_df.to_excel('AStar_m_results.xlsx', index=True)
    # Agv_AStar_results_df.to_excel('Agv_AStar_results.xlsx', index=True)
    # Agv_Jps_Astar_results_df.to_excel('Agv_Jps_Astar_results.xlsx', index=True)


if __name__ == '__main__':
    main()
