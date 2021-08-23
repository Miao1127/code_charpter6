import matplotlib.pyplot as plt
import numpy as np
from file_operator import read_csv
import random
import copy
import math

show_animation = True


def plot(map_data, points, path: list, save_name):
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    ax.invert_yaxis()  # y轴坐标刻度从上到下递增
    ax.xaxis.tick_top()  # x坐标轴置于图像上方
    x = []
    y = []
    point_num = 0
    for point in points:
        x.append(point[0])
        y.append(point[1])
        point_name = '#' + str(point_num)
        plt.text(point[0]+2, point[1]+2, point_name, fontsize=20, family='Times New Roman')  # 给簇首添加编号
        plt.arrow(point[0], point[1], 2*point[2]*np.cos(point[3]), 2*point[2]*np.sin(point[3]), width=0.25, color='r',
                  length_includes_head=True)  # 绘制簇首速度矢量
        for t in range(int(point[4])):
            member_x = int(point[0]) + random.randint(-5, 5)
            member_y = int(point[1]) + random.randint(-5, 5)
            while map_data[member_y][member_x] == 0:
                member_x = int(point[0]) + random.randint(-5, 5)
                member_y = int(point[1]) + random.randint(-5, 5)
            plt.scatter(member_x, member_y, s=30, marker='o', c='y')  # 绘制簇成员
        point_num += 1
    plt.scatter(x, y, s=50, marker='o', c='c')  # 绘制簇首
    if len(path) != 0:
        for _ in range(1, len(path)):   # 从0开始可以形成闭合的路由路径，若从1开始则是首尾不相连的开环路径
            i = path[_ - 1]
            j = path[_]
            plt.arrow(x[i], y[i], x[j] - x[i], y[j] - y[i], width=0.5, color='b', length_includes_head=True)

    ox, oy = load_map(map_data)

    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.tick_params(labelsize=25)
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Times New Roman') for label in labels]
    # 坐标轴范围
    # plt.xlim(0, max(x) * 1.1)
    # plt.ylim(0, max(y) * 1.1)
    plt.savefig(save_name)
    plt.close()


def load_map(m_data):
    """
    读取栅格地图
    :param m_data:
    :return: 障碍物占据栅格行列号
    """
    # 统计障碍物栅格
    o_x, o_y = [], []
    rows, cols = np.array(m_data).shape
    for i in range(rows):
        for j in range(cols):
            if m_data[i][j] == 0:
                o_x.append(j)  # 列号对应x轴
                o_y.append(i)  # 行号对应y轴
    return o_x, o_y


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
    while 1 > x_goal or x_goal > 200 or 1 > y_goal or y_goal > 132:  # 边界
        if 1 > x_goal:
            x_goal = x_goal + 1
        if x_goal > 200:
            x_goal = x_goal - 1
        if 1 > y_goal:
            y_goal = y_goal + 1
        if y_goal > 132:
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
    return [x_goal, y_goal, v, theta, num]


if __name__ == '__main__':
    m_csv = "matrix_map_data.csv"
    save_file_name = 'test.png'
    m_data = np.array(read_csv.run(m_csv))
    swarm = []
    points = []
    path = [16, 19, 18, 17, 11, 12, 6, 8, 7, 9, 1, 3, 2, 0, 4, 5, 10, 13, 14, 15]
    with open('chn.txt') as f:
        for line in f.readlines():
            information = line.split(' ')
            swarm.append(
                dict(index=float(information[0]), x=float(information[1]), y=int(information[2]),
                     v=float(information[3]), theta=float(information[4]), num=float(information[5])))
            points.append([float(information[1]), float(information[2]), float(information[3]), float(information[4]),
                           float(information[5])])
    print(points)
    print(swarm)
    plot(m_data, points, path, save_file_name)
    for i in range(2):
        n_flag = 1
        new_points = []
        new_swarm = []
        for point in points:
            new_point = position_update(point[0], point[1], point[2], point[3], point[4], m_data)
            new_points.append([new_point[0], new_point[1], new_point[2], new_point[3], new_point[4]])
            new_swarm.append(dict(index=n_flag, x=new_point[0], y=new_point[1], v=new_point[2], theta=new_point[3],
                                  num=new_point[4]))
            n_flag += 1
        print(new_points)
        print(swarm)
        save_file_name = str(i) + 'test.png'
        points = copy.deepcopy(new_points)
        swarm = copy.deepcopy(new_swarm)
        plot(m_data, points, path, save_file_name)

