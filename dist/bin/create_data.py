import torch
from matplotlib import pyplot as plt
import numpy as np


LEN = 5
PERSCENT = 0.1 * LEN

with open('xiangxi.txt') as file:
    lists = [s.split(' ') for s in file.read().split('\n')]
    for i, v in enumerate(lists):
        lists[i] = [float(st) for st in v]
    lists = torch.Tensor(lists)
# res = torch.tensor([])
# for i in range(len(lists)//20):
#     temp = lists[i*20:(i+1)*20]
#     if i < 15:
#         res = torch.cat([res, temp[torch.sort(temp[:, 1], dim=0)[1]]], dim=0)
#     else:
#         res = torch.cat([res, temp[torch.sort(temp[:, 0], dim=0)[1]]], dim=0)
sorted_list = lists.view(-1, LEN, 3)
mean = torch.mean(sorted_list, 1)
print(mean)
print(mean.size())
# print(len(res))

# mean = torch.mean(sorted_list, 1)
with open('result.txt', 'w+') as out:
    for i, v in enumerate(mean):
        out.write('%.4f,%.4f,%.4f,' % (v[0], v[1], v[2]))


'''
记录平均值
save_list = lists.view(-1, 10, 4)
mean = torch.mean(save_list, 1)

'''
# with open('result.txt', 'w+') as out:
#     for i, v in enumerate(mean):
#         out.write('%.4f %.4f %.4f %.4f\n' % (v[0], v[1], v[2], v[3]))
