import numpy as np
import time
import pygame

# a=np.zeros((1,1),np.float)
# b=2
# c=a+b
# print(c)
# print(type(c))

import sys

# my_list = [1, 2]
# list_size = len(my_list)
# list_re = my_list[-2]
# print(list_size)
# print(list_re)

# curtime = time.time()
# print(curtime)
# print(type(curtime))

import time

# start = time.time()  # 最低可记录2ms  1993.894577 microseconds
# pygame.time.delay(2)
# end = time.time()
# print(f"time.time() precision: {(end - start) * 1000000:.6f} microseconds")
#
# f = open("test.txt", 'w', encoding='utf-8')
# x = 1
# y = 2
# h = 3
# v = 4
# count = 0
# max_count = 20  # 最大循环次数为 20
#
# while count < max_count:
#     pygame.time.delay(1000)
#     xo = str(x)
#     yo = str(y)
#     ho = str(h)
#     vo = str(v)
#     to = str(time.time())
#     s = xo + ' ' + yo + ' ' + ho + ' ' + vo + ' ' + to + '\n'
#     print(s)
#     f.write(s)
#     x += 1
#     y += 1
#     h += 1
#     v += 1
#     count += 1
#
# f.close()
# print("循环结束")


# # 导入原始数据
# data = np.array([
#     [1, 2, 3, 4, 1722832794.4793248],
#     [1, 2, 3, 4, 1722832795.4795332],
#     [1, 2, 3, 4, 1722832796.4793227],
#     [1, 2, 3, 4, 1722832797.479423],
#     [1, 2, 3, 4, 1722832798.4792597],
#     [1, 2, 3, 4, 1722832799.4792893],
#     [1, 2, 3, 4, 1722832800.4792566],
#     [1, 2, 3, 4, 1722832801.4792826],
#     [1, 2, 3, 4, 1722832802.479257],
#     [1, 2, 3, 4, 1722832803.4792542],
#     [1, 2, 3, 4, 1722832804.4797964],
#     [1, 2, 3, 4, 1722832805.4792721],
#     [1, 2, 3, 4, 1722832806.4797025],
#     [1, 2, 3, 4, 1722832807.4792778],
#     [1, 2, 3, 4, 1722832808.4795516],
#     [1, 2, 3, 4, 1722832809.4793339],
#     [1, 2, 3, 4, 1722832810.4792619],
#     [1, 2, 3, 4, 1722832811.4800372],
#     [1, 2, 3, 4, 1722832812.4794514],
#     [1, 2, 3, 4, 1722832813.4792402]
# ])
#
# # 获取时间序列的开始和结束时间
# start_time = np.min(data[:, 4])
# end_time = np.max(data[:, 4])
#
# # 生成等间隔的时间序列
# time_interval = 0.1  # 时间间隔为 0.1 秒
# time_seq = np.arange(start_time, end_time + time_interval, time_interval)
#
# # 创建新的数据矩阵
# new_data = np.zeros((len(time_seq), data.shape[1]))
# new_data[:, 4] = time_seq
#
# # 将原始数据插入到新的数据矩阵中
# for i, t in enumerate(data[:, 4]):
#     idx = np.searchsorted(time_seq, t)
#     new_data[idx] = data[i]
#
# # 打印结果
# print(new_data)
import numpy as np
from scipy.interpolate import interp1d

# 将数据读取到numpy数组
# data = np.array([
#     [1, 2, 3, 4, 1722834794.3272018],
#     [2, 3, 4, 5, 1722834795.3270361],
#     [3, 4, 5, 6, 1722834796.3274295],
#     [4, 5, 6, 7, 1722834797.3272796],
#     [5, 6, 7, 8, 1722834798.3272343],
#     [6, 7, 8, 9, 1722834799.3278012],
#     [7, 8, 9, 10, 1722834800.3269403],
#     [8, 9, 10, 11, 1722834801.3277001],
#     [9, 10, 11, 12, 1722834802.3269565],
#     [10, 11, 12, 13, 1722834803.3278887],
#     [11, 12, 13, 14, 1722834804.3278804],
#     [12, 13, 14, 15, 1722834805.327428],
#     [13, 14, 15, 16, 1722834806.326962],
#     [14, 15, 16, 17, 1722834807.3273344],
#     [15, 16, 17, 18, 1722834808.3269622],
#     [16, 17, 18, 19, 1722834809.3274975],
#     [17, 18, 19, 20, 1722834810.3274567],
#     [18, 19, 20, 21, 1722834811.3275464],
#     [19, 20, 21, 22, 1722834812.3271956],
#     [20, 21, 22, 23, 1722834813.3272793]
# ])
#
# # 提取时间戳和数据
# x = data[:, 4]
# y = data[:, 0:4]
#
# # 创建插值函数
# f = interp1d(x, y, kind='linear', axis=0)
#
# # 生成新的时间戳
# new_x = np.arange(x[0], x[-1], 1)
#
# # 进行插值
# new_y = f(new_x)
#
# # 输出结果
# print(new_y)
# t_stamp = 1.5
# t_temp = [0, 4, 8, 12, 16, 20]
# x_temp = [0, 2, 4, 6, 8, 10]
# new_t_temp = np.arange(t_temp[0], t_temp[-1] + 1, t_stamp)
# print(new_t_temp)
# f_x = interp1d(t_temp, x_temp, kind='linear')
# new_x_temp = f_x(new_t_temp)
# print(new_x_temp)
import datetime

# 获取当前日期和时间
now = datetime.datetime.now()

# 使用当前日期和时间作为文件名
# save_dir = 'D:/my_files'
file_name = f'log{now.strftime("%Y%m%d_%H%M%S")}.txt'

testlist = []
a = 1.
b = 2.3
c = 0.6
d = True
testlist.append(a)
testlist.append(b)
testlist.append(c)
testlist.append(d)
print(testlist)

file = open(file_name, 'w')
for item in testlist:
    file.write(str(item) + ' ')
# file.write('这是第二行内容。\n')

# 关闭文件
file.close()


