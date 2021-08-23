# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/39:11
# 文件名：main.py
# 开发工具：PyCharm
# 功能：分区间簇首中继通信路由算法

import math
from aco_solver.aco import ACO, Graph
from aco_solver.plot import plot
import time
import matplotlib.pyplot as plt


def distance(usv1: dict, usv2: dict):
    return math.sqrt((usv1['x'] - usv2['x']) ** 2 + (usv1['y'] - usv2['y']) ** 2)


def main(file):
    start = time.time()
    swarm = []
    points = []
    with open('./data/'+file) as f:
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
    plot(points, path)
    return time.time() - start, cost


if __name__ == '__main__':
    x = []
    y = []
    y2 = []
    t, cost = main('chn10.txt')
    y.append(t)
    x.append(10)
    y2.append(cost)

    print(y)
    plt.plot(x, y)
    plt.show()
    plt.plot(x, y2)
    plt.show()
