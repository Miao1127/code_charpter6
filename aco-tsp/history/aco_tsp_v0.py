# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：runlong
# 开发时间：2020/12/510:36
# 文件名：aco_tsp_v0.py
# 开发工具：PyCharm
# 功能：蚁群算法计算全局通信路由

from aco_solver.aco import ACO, Graph
from aco_solver.plot import plot
import time
import matplotlib.pyplot as plt
from aco_solver import path_distance, a_star
from file_operator import read_csv, makedir
from aco_solver import zone_plot
import numpy as np


def distance(usv1: dict, usv2: dict):
    start = [int(usv1['y']), int(usv1['x'])]           # 起点和终点以行列编号输入
    goal = [int(usv2['y']), int(usv2['x'])]
    print('Start: %s' % start)
    print('Goal: %s' % goal)
    if start == goal:
        d = 0
    else:
        a = a_star.AStar(m_data, start, goal, 1000000)
        a.run()
        path = a.path_backtrace()
        d = path_distance.run(path[0], path[1])  # path[0]存储路径行号, path[1]存储路径列号
    print('Distance: %s' % d)
    return d


def main(file, s_name):
    start = time.time()
    swarm = []
    points = []
    with open(file) as f:
        for line in f.readlines():
            information = line.split(' ')
            swarm.append(dict(index=int(information[0]), x=int(information[1]), y=int(information[2])))
            points.append((int(information[1]), int(information[2])))
    cost_matrix = []
    rank = len(swarm)
    for i in range(rank):
        row = []
        for j in range(rank):
            row.append(distance(swarm[i], swarm[j]))
        cost_matrix.append(row)
    aco = ACO(10, 100, 1.0, 10.0, 0.5, 10, 2)
    graph = Graph(cost_matrix, rank)
    path, cost = aco.solve(graph, points)
    print('cost: {}, path: {}'.format(cost, path))
    plot(m_data, points, path, s_name)
    return time.time() - start, cost


if __name__ == '__main__':
    start_time = time.time()
    save_path = 'E:/博士论文试验数据/chapter6/aco_tsp/' + str(int(start_time)) + '/'
    makedir.mkdir(save_path)
    save_file_name = save_path + 'result.png'
    m_csv = "matrix_map_data.csv"
    m_data = np.array(read_csv.run(m_csv))
    fig = plt.figure(figsize=(21, 14))
    ax = fig.add_subplot(111)
    # 加载地图数据
    ox, oy = zone_plot.load_map(m_data)
    x = []
    y = []
    y2 = []
    t, cost = main('chn.txt', save_file_name)
    y.append(t)
    x.append(10)
    y2.append(cost)

    print(y)
    plt.plot(x, y)
    plt.show()
    plt.plot(x, y2)
    plt.show()
