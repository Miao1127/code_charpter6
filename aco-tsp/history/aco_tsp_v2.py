# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/720:23
# 文件名：aco_tsp_v2.py
# 开发工具：PyCharm
# 功能：链路开销为最简单形式的欧式距离，弊端：形成的通信链路可能存在穿越障碍物的情况，一旦出现此情况，在现实中将会影响通信信号的稳定传输

from aco_solver.aco import ACO, Graph
from aco_solver.plot import plot
import time
from aco_solver import path_distance, a_star
from file_operator import read_csv, makedir
import numpy as np
import random
import copy
import math


def distance(usv1: dict, usv2: dict):
    d = math.sqrt((usv1['x'] - usv2['x']) ** 2 + (usv1['y'] - usv2['y']) ** 2)
    p = []
    if d == 0:
        d = 0.001
    return [d, p]


def read_start_points(file):
    swarm = []
    points = []
    with open(file) as f:
        for line in f.readlines():
            information = line.split(' ')
            swarm.append(
                dict(index=float(information[0]), x=float(information[1]), y=int(information[2]),
                     v=float(information[3]), theta=float(information[4]), num=float(information[5])))
            points.append([float(information[1]), float(information[2]), float(information[3]), float(information[4]),
                           float(information[5])])
    return swarm, points


# 当前usv运动模拟
def position_update(x, y, v, theta, num, m_data):
    x_goal = x + v * np.cos(theta)
    y_goal = y + v * np.sin(theta)

    x_goal = int(x_goal)
    y_goal = int(y_goal)

    n = 0
    neighbour_list = [[x_goal + 3, y_goal], [x_goal + 3, y_goal - 1], [x_goal + 3, y_goal - 2],
                      [x_goal + 3, y_goal - 3], [x_goal + 2, y_goal - 3], [x_goal + 1, y_goal - 3],
                      [x_goal, y_goal - 3], [x_goal - 1, y_goal - 3], [x_goal - 2, y_goal - 3],
                      [x_goal - 3, y_goal - 3], [x_goal - 3, y_goal - 2], [x_goal - 3, y_goal - 1],
                      [x_goal - 3, y_goal], [x_goal - 3, y_goal + 1], [x_goal - 3, y_goal + 2],
                      [x_goal - 3, y_goal + 3], [x_goal - 2, y_goal + 3], [x_goal - 1, y_goal + 3],
                      [x_goal, y_goal + 3], [x_goal + 1, y_goal + 3], [x_goal + 2, y_goal + 3],
                      [x_goal + 3, y_goal + 3], [x_goal + 3, y_goal + 2], [x_goal + 3, y_goal + 1]]
    while 5 > x_goal or x_goal > 195 or 5 > y_goal or y_goal > 127:  # 边界
        if 5 > x_goal:
            x_goal = x_goal + 1
        if x_goal > 195:
            x_goal = x_goal - 1
        if 5 > y_goal:
            y_goal = y_goal + 1
        if y_goal > 127:
            y_goal = y_goal - 1
    while m_data[y_goal][x_goal] == 0:   # 障碍物
        x_goal = neighbour_list[n][0]
        y_goal = neighbour_list[n][1]
        n += 1

    if n != 0:
        theta = math.atan2(y_goal-y, x_goal-x)
        v = 2
    elif n == 0:
        theta = theta + 0.25
        v = 3
    return [x_goal, y_goal, v, theta, num]


def main(file):
    swarm, points = read_start_points(file)
    for t in range(simulation_time):
        cost_matrix = []
        rank = len(swarm)
        path_dict = []
        for i in range(rank):
            row = []
            for j in range(rank):
                if i == j:
                    row.append(0)
                    path_dict.append(
                        dict(index=str(i) + '-' + str(j), path=[]))
                else:
                    [d, p] = distance(swarm[i], swarm[j])
                    row.append(d)
                    path_dict.append(
                        dict(index=str(i)+'-'+str(j), path=p))
            cost_matrix.append(row)
        aco = ACO(10, 100, 1.0, 10.0, 0.5, 10, 2)
        graph = Graph(cost_matrix, rank)
        path, cost = aco.solve(graph, points)
        path_2 = path + path
        new_path = []
        for i in range(len(path_2)):  # 设置起始USV编号
            if path_2[i] == 0:
                new_path = path_2[i:i + len(path)]
                break
        print('cost: {}, path: {}'.format(cost, new_path))
        time_flag = str(int(time.time()))
        s_name = save_path + time_flag + '.png'
        plot(m_data, points, new_path, s_name)
        n_flag = 1
        new_points = []
        new_swarm = []
        for point in points:
            new_point = position_update(point[0], point[1], point[2], point[3], point[4], m_data)
            new_points.append([new_point[0], new_point[1], new_point[2], new_point[3], new_point[4]])
            new_swarm.append(dict(index=n_flag, x=new_point[0], y=new_point[1], v=new_point[2], theta=new_point[3],
                                  num=new_point[4]))
            n_flag += 1
        points = copy.deepcopy(new_points)
        swarm = copy.deepcopy(new_swarm)


if __name__ == '__main__':
    start_time = time.time()
    save_path = 'E:/博士论文试验数据/chapter6/aco_tsp/' + str(int(start_time)) + '/'
    makedir.mkdir(save_path)
    m_csv = "matrix_map_data.csv"
    m_data = np.array(read_csv.run(m_csv))
    simulation_time = 30  # 仿真循环次数
    main('chn.txt')
