# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/414:21
# 文件名：kmedoids_v1.py
# 开发工具：PyCharm
# 功能：k中心聚类

import time
import random
from sklearn.datasets import make_blobs
from file_operator import makedir
import pandas as pd
from pyclust import KMedoids
import numpy as np
from sklearn.manifold import TSNE
import matplotlib.pyplot as plt


def generate_data(n_points, centroid_num, n_features):
    """
    生成数据
    :param n_points: 生成数据的数量
    :param centroid_num: 生成数据的中心点数量
    :param n_features: 数据维度
    :return:usv位置数据
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
    plt.scatter(data[:, 0], data[:, 1])
    plt.title("generate data")
    # plt.show()
    # 保存数据
    # 保存数据
    for i in range(len(data)):
        data2csv = pd.DataFrame([data[i]])
        data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
    # 读取数据
    # data = np.loadtxt('./results/usv_position')
    return data.tolist()


def initial_center(k, u_position):
    usv_num = len(u_position)  # usv数目
    usv_list = list(range(usv_num))  # usv编号列表
    center_list = []  # 中心节点列表
    # 1. 计算各个usv间的距离，以矩阵形式记录
    distance_matrix = np.zeros((usv_num, usv_num))  # 距离矩阵
    for i in range(usv_num):
        for j in range(usv_num):
            if i != j:
                distance_matrix[i][j] = np.sqrt(
                    (u_position[i][0] - u_position[j][0]) ** 2 + (u_position[i][1] - u_position[j][1]) ** 2)
    # 2.随机选择第一个中心节点
    center_first = random.randint(0, usv_num - 1)
    usv_list.remove(center_first)  # 从usv编号列表中将此usv去掉
    center_list.append(center_first)  # 将此中心节点添加到中心节点列表中
    # 3.根据概率选择其他中心节点
    for kk in range(k-1):
        p_list = np.zeros(usv_num)
        distance_to_center = np.zeros(usv_num)
        for i in usv_list:
            for j in center_list:
                distance_to_center[i] += distance_matrix[i][j]
        for i in range(len(p_list)):
            p_list[i] = distance_to_center[i] / sum(distance_to_center)
        new_center = random_pick(usv_list, p_list)
        center_list.append(new_center)
        usv_list.remove(new_center)
    return center_list


def random_pick(some_list, probabilities):
    """
    轮盘赌形式选择元素
    :param some_list: 用于选择的列表元素
    :param probabilities: 列表元素被选择的概率区间
    :return: 被选择元素
    """
    x = random.uniform(0, 1)
    cumulative_probability = 0.0
    for item, item_probability in zip(some_list, probabilities):
        cumulative_probability += item_probability
        if x < cumulative_probability:
            break
    return item


def j0(n_bit, u_position):
    """
    簇首间发送信息能耗
    :param n_bit: 发送的信息字节数
    :param u_position: 各个usv位置
    :return:
    """
    j0_sum = 0
    for usv1 in u_position:
        max_distance = 0
        for usv2 in u_position:
            new_distance = (usv1[0] - usv2[0]) ** 2 + (
                    usv1[0] - usv2[0]) ** 2
            if new_distance > max_distance:
                max_distance = new_distance
        if max_distance != 0:
            j0_sum += n_bit * (50 + 100 * max_distance + (len(u_position)-1)*50)
    return j0_sum


def j1(n_bit, cluster, u_position):
    """
    节点向所在的簇的簇首发送信息能耗
    :param n_bit: 发送的信息字节数
    :param cluster: 字典形式的簇，字典的键为簇首usv编号，值为簇内成员
    :param u_position: 各个usv位置
    :return:
    """
    j1_sum = 0
    for key in cluster:
        for num in cluster[key]:
            j1_sum += n_bit * (50 + 100 * ((u_position[key][0] - u_position[num][0]) ** 2 + (
                        u_position[key][0] - u_position[num][0]) ** 2) + 50)
    return j1_sum


def j2(n_bit, cluster, u_position):
    """
    簇首间发送信息能耗
    :param n_bit: 发送的信息字节数
    :param cluster: 字典形式的簇，字典的键为簇首usv编号，值为簇内成员
    :param u_position: 各个usv位置
    :return:
    """
    j2_sum = 0
    for key in cluster:
        max_distance = 0
        for num in cluster:
            new_distance = (u_position[key][0] - u_position[num][0]) ** 2 + (
                    u_position[key][0] - u_position[num][0]) ** 2
            if new_distance > max_distance:
                max_distance = new_distance
        if max_distance != 0:
            j2_sum += (len(cluster[key])+1)*n_bit * (50 + 100 * max_distance + (len(cluster)-1)*50)
    return j2_sum


def j3(n_bit, cluster, u_position):
    """
    簇首向簇内成员节点发送信息能耗
    :param n_bit: 发送的信息字节数
    :param cluster: 字典形式的簇，字典的键为簇首usv编号，值为簇内成员
    :param u_position: 各个usv位置
    :return:
    """
    j3_sum = 0
    for key in cluster:
        max_distance = 0
        for num in cluster[key]:
            new_distance = (u_position[key][0] - u_position[num][0]) ** 2 + (
                    u_position[key][0] - u_position[num][0]) ** 2
            if new_distance > max_distance:
                max_distance = new_distance
        j3_sum += (len(u_position)-len(cluster[key])-1)*n_bit * (50 + 100 * max_distance + 50)
    return j3_sum


def main():
    save_file_name = save_path + 'result.csv'
    # 1.生成数据
    # with open("data", "rb") as f:
    #     data = pickle.load(f)
    data = generate_data(usv_points, centroid_number, usv_position_features)  # 生成数据
    data = np.array(data)
    for usv_num in range(15, usv_points):
        usv_position = np.array(data[:usv_num])
        # 4.迭代中心聚类
        j_list = []        # 记录所有聚类形式的能耗
        j_min = np.inf     # 记录最小能耗
        min_num = 0        # 记录最小能耗对应的聚类簇数目
        for i in range(auv_num, int(usv_num/3)):
            k_m = KMedoids(n_clusters=i, distance='euclidean', max_iter=10)
            lab = k_m.fit_predict(usv_position)
            centers = k_m.centers_
            cluster_dict = {}
            for j in range(len(centers)):
                cluster_member = [i for i, x in enumerate(lab) if x == j]
                usv_data = usv_position.tolist()
                c = centers[j].tolist()
                center_num = usv_data.index(c)
                cluster_dict[center_num] = cluster_member
            j_1 = j1(send_bit, cluster_dict, usv_position)
            j_2 = j2(send_bit, cluster_dict, usv_position)
            j_3 = j3(send_bit, cluster_dict, usv_position)
            j_list.append(j_1+j_2+j_3)
            if j_1+j_2+j_3 < j_min:
                j_min = j_1+j_2+j_3
                min_num = i
        j_0 = j0(send_bit, usv_position)
        # print(j_list)
        print(min_num)
        print(j_min)
        print(j_0)
        data = [usv_num, min_num, j_min, j_0, 100*(j_0-j_min)/j_0]
        data2csv = pd.DataFrame([data])
        data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
    # colors = ([['red', 'blue', 'black', 'yellow', 'green'][i] for i in k])
    # plt.subplot(219 + i)
    # plt.scatter(data_TSNE[:, 0], data_TSNE[:, 1], c=colors, s=10)
    # plt.title('K-medoids Resul of '.format(str(i)))
    # plt.show()


if __name__ == '__main__':
    # 数据保存路径
    start_time = time.time()
    save_path = 'E:/博士论文试验数据/chapter6/' + str(int(start_time)) + '/'
    makedir.mkdir(save_path)
    usv_points = 50            # 生成的数据的个数
    centroid_number = 20       # 簇中心的数量
    usv_position_features = 2  # 数据维度
    auv_num = 3                # auv数目
    send_bit = 10
    main()



