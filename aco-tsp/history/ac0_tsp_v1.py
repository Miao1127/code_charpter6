# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/710:19
# 文件名：ac0_tsp_v1.py
# 开发工具：PyCharm
# 功能：usv加入速度项

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
    start = [int(usv1['y']), int(usv1['x'])]           # 起点和终点以行列编号输入
    goal = [int(usv2['y']), int(usv2['x'])]
    print('Start: %s' % start)
    print('Goal: %s' % goal)
    if start == goal:
        d = 0
        p = []
    else:
        a = a_star.AStar(m_data, start, goal, 1000000)
        a.run()
        p = a.path_backtrace()
        d = path_distance.run(p[0], p[1])  # path[0]存储路径行号, path[1]存储路径列号
    print('Distance: %s' % d)
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
def position_update(x, y, v, theta, m_data):
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
    while 5 > x_goal or x_goal > 195 or 5 > y_goal or y_goal > 128:  # 边界
        if 5 > x_goal:
            x_goal = x_goal + 1
        if x_goal > 195:
            x_goal = x_goal - 1
        if 5 > y_goal:
            y_goal = y_goal + 1
        if y_goal > 128:
            y_goal = y_goal - 1
    while m_data[y_goal][x_goal] == 0:   # 障碍物
        x_goal = neighbour_list[n][0]
        y_goal = neighbour_list[n][1]
        n += 1

    if n != 0:
        theta = math.atan2(y_goal-y, x_goal-x)
        v = random.uniform(1, 2)
    elif n == 0:
        theta = theta + random.uniform(-0.5, 0.5)
        v = random.uniform(2, 3)
    return [x_goal, y_goal, v, theta]


def main(file):
    swarm, points = read_start_points(file)
    for t in range(simulation_time):
        cost_matrix = []
        rank = len(swarm)
        path_dict = []
        for i in range(rank):
            row = []
            for j in range(rank):
                row.append(distance(swarm[i], swarm[j])[0])
                path_dict.append(
                    dict(index=str(i)+'-'+str(j), path=distance(swarm[i], swarm[j])[1]))
            cost_matrix.append(row)
        aco = ACO(10, 100, 1.0, 10.0, 0.5, 10, 2)
        graph = Graph(cost_matrix, rank)
        path, cost = aco.solve(graph, points)
        print('cost: {}, path: {}'.format(cost, path))
        time_flag = str(int(time.time()))
        s_name = save_path + time_flag + '.png'
        plot(m_data, points, path, s_name)
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
    simulation_time = 100  # 仿真循环次数
    main('chn.txt')
