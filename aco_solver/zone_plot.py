# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/914:57
# 文件名：zone_plot.py
# 开发工具：PyCharm
# 功能：绘制分区划分效果图

# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：runlong
# 开发时间：2019/11/314:33
# 文件名：vertex_plot.py
# 开发工具：PyCharm
# 功能：绘制分区顶点

import matplotlib.pyplot as plt
import numpy as np
from file_operator import read_csv
import time


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


def load_grid(g_dict):
    """
    读取栅格地图
    :param g_dict:
    :return: 分区顶点占据栅格行列号
    """
    x, y = [], []
    for kk in range(len(g_dict)):
        g_key = '#' + str(kk + 1)         # 按照字典键大小顺序统计
        rows, cols = g_dict[g_key].shape
        for ii in range(rows):
                x.append(g_dict[g_key][ii][1])  # 列号
                y.append(g_dict[g_key][ii][0])  # 行号
    return x, y


if __name__ == '__main__':
    show_animation = True
    # 1.加载障碍物地图
    map_data = read_csv.run('../aco-tsp/history/matrix_map_data.csv')
    ox, oy = load_map(map_data)
    swarm = []
    points = []
    with open('chn.txt') as f:
        for line in f.readlines():
            information = line.split(' ')
            swarm.append(dict(index=int(information[0]), x=int(information[1]), y=int(information[2])))
            points.append([int(information[1]), int(information[2])])
    print(points)
    points = np.array(points)
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)
    while True:
        if show_animation:
            plt.cla()  # 清除当前图形中的当前活动轴，其他轴不受影响
            ax.invert_yaxis()        # y轴坐标刻度从上到下递增
            ax.xaxis.tick_top()      # x坐标轴置于图像上方
            plt.plot(ox, oy, ".k")
            plt.grid(True)
            plt.tick_params(labelsize=25)
            labels = ax.get_xticklabels() + ax.get_yticklabels()
            [label.set_fontname('Times New Roman') for label in labels]
            for point in points:
                plt.plot(point[0], point[1], '*r')  # 绘制静态目标点
                plt.pause(1)


