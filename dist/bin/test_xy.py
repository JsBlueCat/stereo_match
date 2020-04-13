import torch
from matplotlib import pyplot as plt
import numpy as np


LEN = 5
PERSCENT = 0.1 * LEN

with open('out.txt') as file:
    lists = [s.split(' ') for s in file.read().split('\n')]
    for i, v in enumerate(lists):
        lists[i] = [float(st) for st in v]
    lists = torch.Tensor(lists)

sorted_list = lists.view(-1, LEN, 4)
res = torch.mean(sorted_list, 1)
print(res)

'''
记录平均值
save_list = lists.view(-1, 10, 4)
mean = torch.mean(save_list, 1)

'''
with open('result.txt', 'w+') as out:
    for i, data in enumerate(res):
        if i%10 == 2 or i%10 == 5 or i%10 == 9:
            out.write('%.4f %.4f %.4f\n' % (data[0], data[1], data[2]))
    out.write('\n')
    sfafas 
    Z_BATCH_SIZE = 8  # Z轴移动的组数
    Z_BATCH_DISTANCE = 500  # Z轴移动的间距
    COL_SIZE = 5  # 横向移动多少组
    COL_DISTANCE = 50  # 横向移动的间距
    ROW_SIZE = 2  # 纵向移动多少组
    ROW_DISTANCE = 100  # 纵向移动的间距
    for z in range(Z_BATCH_SIZE):
        for row in range(ROW_SIZE):
            for col in range(COL_SIZE):
                out.write('%.4f %.4f %.4f\n' % (COL_DISTANCE*col-100,
                                                ROW_DISTANCE*row, Z_BATCH_DISTANCE*z+995))
