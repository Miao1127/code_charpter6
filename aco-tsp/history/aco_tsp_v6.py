# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/99:07
# 文件名：aco_tsp_v6.py
# 开发工具：PyCharm
# 功能：仅考虑Astar算法计算等效距离，添加记录稳定通信时间和距离功能

from aco_solver.aco import ACO, Graph
from aco_solver.plot import plot
import time
from aco_solver import path_distance, a_star
from file_operator import read_csv, makedir
import numpy as np
import random
import copy
import math
import pandas as pd


def distance(usv1: dict, usv2: dict):
    start = [int(usv1['y']), int(usv1['x'])]           # 起点和终点以行列编号输入
    goal = [int(usv2['y']), int(usv2['x'])]
    index1 = int(usv1['index'])
    index2 = int(usv2['index'])
    print('Start: %s' % start)
    print('Goal: %s' % goal)
    if index1 == index2:
        d = 0
        p = []
    else:
        a = a_star.AStar(m_data, start, goal, 1000000)
        a.run()
        p = a.path_backtrace()
        d = path_distance.run(p[0], p[1])  # path[0]存储路径行号, path[1]存储路径列号
        if d == 0:
            d = 0.001
    print('Distance: %s' % d)
    return [d, p]


# 计算簇数目列表
def cluster_num(points):
    points_num = len(points)
    n_list = np.zeros(points_num)
    for i in range(points_num):
        n_list[i] = points[i][4]
    return n_list


# 计算稳定通信时间矩阵
def stable_time(points, r_min):
    points_num = len(points)
    t_matrix = np.zeros((points_num, points_num))
    for i in range(points_num):
        for j in range(points_num):
            if i != j:
                x1 = points[i][0]
                y1 = points[i][1]
                v1 = points[i][2]
                theta1 = points[i][3]
                x2 = points[j][0]
                y2 = points[j][1]
                v2 = points[j][2]
                theta2 = points[j][3]

                a = v1 * math.cos(theta1) - v2 * math.cos(theta2)
                b = x1 - x2
                c = v1 * math.sin(theta1) - v2 * math.sin(theta2)
                d = y1 - y2
                if a**2+c**2 == 0:
                    print(v1, v2, theta1, theta2)
                    print(a, c)
                t_matrix[i][j] = (-(a*b + c*d) + math.sqrt((a**2 + c**2)*r_min**2-(a*d+b*c)**2))/(a**2+c**2)
    return t_matrix


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
    while 8 > x_goal or x_goal > 192 or 8 > y_goal or y_goal > 124:  # 边界
        if 8 > x_goal:
            x_goal = x_goal + 1
        if x_goal > 192:
            x_goal = x_goal - 1
        if 8 > y_goal:
            y_goal = y_goal + 1
        if y_goal > 124:
            y_goal = y_goal - 1
    while m_data[y_goal][x_goal] == 0:  # 障碍物
        x_goal = neighbour_list[n][0]
        y_goal = neighbour_list[n][1]
        n += 1
    while m_data[y_goal][x_goal] == 0:   # 障碍物
        x_goal = neighbour_list[n][0]
        y_goal = neighbour_list[n][1]
        n += 1

    if n != 0:
        theta = math.atan2(y_goal - y, x_goal - x)
        v = 2
    elif n == 0:
        theta = theta + 0.1
        if theta > 3.14:
            theta = theta - 0.1
        v = 3
    return [x_goal, y_goal, v, theta, num]


def main(file):
    swarm, points = read_start_points(file)
    data2csv = pd.DataFrame([['t', 'stable_time', 'cluster_num']])
    data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
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
        for i in range(len(path_2)):
            if path_2[i] == 0:
                new_path = path_2[i:i+len(path)]
                break
        print('cost: {}, path: {}'.format(cost, path))
        t_m = stable_time(points, r)
        n_l = cluster_num(points)
        stable_time_10 = 0
        cluster_num_10 = 0
        for i in range(10):
            row = int(new_path[i])
            col = int(new_path[i + 1])
            stable_time_10 += t_m[row, col]
            cluster_num_10 += n_l[col]
        save_data = [t, stable_time_10, cluster_num_10]
        data2csv = pd.DataFrame([save_data])
        data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
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
    save_file_name = save_path + 'stable_time_and_cluster_num.csv'
    m_csv = "matrix_map_data.csv"
    m_data = np.array(read_csv.run(m_csv))
    simulation_time = 50  # 仿真循环次数
    r = 200               # 广播范围
    main('chn.txt')
