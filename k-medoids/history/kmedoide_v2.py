# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：20/12/610:04
# 文件名：kmedoide_v2.py
# 开发工具：PyCharm
# 功能：k中心聚类


import time
from sklearn.datasets import make_blobs
from file_operator import makedir, read_csv
import pandas as pd
from pyclust import KMedoids
import numpy as np
import matplotlib.pyplot as plt


def generate_data(n_points, centroid_num, n_features):
    """
    生成数据
    :param n_points: 生成数据的数量
    :param centroid_num: 生成数据的中心点数量
    :param n_features: 数据维度
    :return:usv位置数据
    """
    save_file_name = save_path + 'usv_position.csv'
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


def j0(n_bit, u_position):
    """
    簇首间发送信息能耗
    :param n_bit: 发送的信息字节数
    :param u_position: 各个usv位置
    :return:
    """
    j0_sum = 0
    # 以距离最远的usv为最大无线电广播范围
    # for usv1 in u_position:
    #     max_distance = 0
    #     for usv2 in u_position:
    #         new_distance = (usv1[0] - usv2[0]) ** 2 + (
    #                 usv1[0] - usv2[0]) ** 2
    #         if new_distance > max_distance:
    #             max_distance = new_distance
    #     if max_distance != 0:
    #         j0_sum += n_bit * (50e-9 + 100e-12 * max_distance*1000 + (len(u_position)-1)*50e-9)
    # 以最大广播功率发送信息
    for i in range(len(u_position)):
        for j in range(len(u_position)):
            if i != j:
                j0_sum += n_bit * (50e-9 + 100e-12 * 10*1000 + (len(u_position)-1)*50e-9)
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
            j1_sum += n_bit * (50e-9 + 100e-12 * ((u_position[key][0] - u_position[num][0]) ** 2 + (
                        u_position[key][0] - u_position[num][0]) ** 2) * 1000 + 50e-9)
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
            j2_sum += len(cluster[key])*n_bit * (50e-9 + 100e-12 * max_distance * 1000 + (len(cluster)-1)*50e-9)
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
        j3_sum += len(u_position)*n_bit * (50e-9 + 100e-12 * max_distance * 1000 + (len(cluster[key])-1)*50e-9)
    return j3_sum


def main():
    save_file_name = save_path + 'result.csv'
    save_data = ['usv_num', 'cluster_num', 'cost(kj)', 'broadcast_cost(kj)', 'save_rate(%)']
    data2csv = pd.DataFrame([save_data])
    data2csv.to_csv(save_file_name, mode='a', header=False, index=None)
    # 1.第一种，生成数据
    # data = generate_data(usv_points, centroid_number, usv_position_features)  # 生成数据
    # 2.第二种，读取已有数据
    u_data = read_csv.run('E:/博士论文试验数据/chapter6/k-medoide/1个AUV5分钟/100usv_position.csv')
    u_data = np.array(u_data)
    data = u_data[:, 1:3]
    for usv_num in range(auv_num*3, usv_points+1):
        usv_position = data[:usv_num]
        # 4.迭代中心聚类
        j_list = []        # 记录所有聚类形式的能耗
        j_min = np.inf     # 记录最小能耗
        min_num = 0        # 记录最小能耗对应的聚类簇数目
        lab_final = []
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
            j_1 = j1(send_bit, cluster_dict, usv_position)/1000
            j_2 = j2(send_bit, cluster_dict, usv_position)/1000
            j_3 = j3(send_bit, cluster_dict, usv_position)/1000
            j_list.append(j_1+j_2+j_3)
            if j_1+j_2+j_3 < j_min:
                j_min = (j_1+j_2+j_3)
                min_num = i
                lab_final = lab
        j_0 = j0(send_bit, usv_position)/1000
        # print(j_list)
        print(min_num)
        print(j_min)
        print(j_0)
        save_data = [usv_num, min_num, j_min, j_0, 100*(j_0-j_min)/j_0]
        data2csv = pd.DataFrame([save_data])
        data2csv.to_csv(save_file_name, mode='a', header=False, index=None)

        save_usv_position = save_path + str(usv_num) + 'usv_position.csv'
        for i in range(len(lab_final)):
            save_data = [lab_final[i], usv_position[i][0], usv_position[i][1]]
            data2csv = pd.DataFrame([save_data])
            data2csv.to_csv(save_usv_position, mode='a', header=False, index=None)
    # colors = ([['red', 'blue', 'black', 'yellow', 'green'][i] for i in k])
    # plt.subplot(219 + i)
    # plt.scatter(data_TSNE[:, 0], data_TSNE[:, 1], c=colors, s=10)
    # plt.title('K-medoids Resul of '.format(str(i)))
    # plt.show()


if __name__ == '__main__':
    # 数据保存路径
    start_time = time.time()
    for i in range(1, 21):
        usv_points = 100                      # 生成的数据的个数
        centroid_number = 100                  # 簇中心的数量
        usv_position_features = 2             # 数据维度
        auv_num = i                          # auv数目
        print('AUV number:%s' % auv_num)
        save_path = 'E:/博士论文试验数据/chapter6/k-medoide/' + str(int(start_time)) + '/' + str(auv_num) + '/'
        makedir.mkdir(save_path)
        swarm_data = 264 * 8 * 5              # 心跳消息包，频率为5hz
        decision_data = 264 * 8 * 0.1         # 决策消息包，频率为0.1hz
        detection_data = 31 * 8 * 2           # 探测消息包，频率为2hz
        distance_data = (auv_num+11) * 8 * 2  # 水下目标距离信息，频率为2hz
        t_step = 5*60                        # 簇维持周期
        send_bit = (swarm_data + decision_data + detection_data + distance_data) * t_step
        main()




