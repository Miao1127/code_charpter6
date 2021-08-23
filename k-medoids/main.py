# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/38:46
# 文件名：main.py
# 开发工具：PyCharm
# 功能：分区内usv节点聚类

import time
import multiprocessing
from multiprocessing import cpu_count
import random
from solver import kmedoids
import numpy as np
from sklearn.datasets import make_blobs
from matplotlib import pyplot
import json
import os
from file_operator import makedir
import pandas as pd
import heapq


def generate_data(n_points, centroid_num, n_features):
    """
    生成数据
    :param n_points: 生成数据的数量
    :param centroid_num: 生成数据的中心点数量
    :param n_features: 数据维度
    :return:
    """
    save_file_name = save_path + 'usv_position_interval.csv'
    # # 第一种数据生成方式,可选择聚类形式
    data, target = make_blobs(n_samples=n_points, n_features=n_features, centers=centroid_num, center_box=(-10, 10))
    # 添加噪声
    # np.put(data, [n_points, 0], 1, mode='clip')
    # np.put(data, [n_points, 1], 1, mode='clip')
    # 第二种数据生成方式
    # data = np.zeros((n_points, 2))
    # row, col = data.shape
    # for i in range(row):
    #     for j in range(col):
    #         data[i][j] = np.random.uniform(-10, 10)
    # 画图
    pyplot.scatter(data[:, 0], data[:, 1])
    pyplot.title("generate data")
    pyplot.show()
    # 保存数据
    # 保存数据
    for i in range(len(data)):
        data2csv = pd.DataFrame([data[i]])
        data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
    # 读取数据
    # data = np.loadtxt('./results/usv_position')
    return data.tolist()


def initial_k(u_position):
    usv_num = len(u_position)           # usv数目
    usv_list = list(range(usv_num))     # usv编号列表
    center_list = []                    # 中心节点列表
    cluster_list = []                   # 簇成员列表
    # 1. 计算各个usv间的距离，以矩阵形式记录
    distance_matrix = np.zeros((usv_num, usv_num))  # 距离矩阵
    for i in range(usv_num):
        for j in range(usv_num):
            if i != j:
                distance_matrix[i][j] = np.sqrt(
                    (u_position[i][0] - u_position[j][0]) ** 2 + (u_position[i][1] - u_position[j][1]) ** 2)
    # 2.随机选择一个usv作为第一个初始中心节点
    center_first = random.randint(0, usv_num-1)
    usv_list.remove(center_first)                # 从usv编号列表中将此usv去掉
    center_list.append(center_first)             # 将此中心节点添加到中心节点列表中
    distance_data = distance_matrix[:, center_first].tolist()
    # 3.距初始中心节点的距离最小的3个usv的索引
    min_num_index_list = map(distance_data.index, heapq.nsmallest(3, distance_data))







def main():
    # 1.生成数据
    # with open("data", "rb") as f:
    #     data = pickle.load(f)
    data = generate_data(usv_points, centroid_number, usv_position_features)  # 生成数据
    pool = multiprocessing.Pool(processes=cpu_count())                        # 进程池
    # 2.确定k值范围

    # 3.初始化聚类中心

    # 4.迭代中心聚类


if __name__ == '__main__':
    # 数据保存路径
    start_time = time.time()
    save_path = 'E:/博士论文试验数据/chapter4/' + str(int(start_time)) + '/'
    makedir.mkdir(save_path)
    usv_points = 50  # 生成的数据的个数
    centroid_number = 30  # 簇中心的数量
    usv_position_features = 2  # 数据维度
    main()


