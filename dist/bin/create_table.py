import torch
from matplotlib import pyplot as plt
import numpy as np

with open('out.txt') as file:
    lists = [s.split(' ') for s in file.read().split('\n')]
    for i, v in enumerate(lists):
        lists[i] = [float(st) for st in v]
    lists = torch.Tensor(lists)
# print(lists.size())
'''
记录平均值
'''
save_list = lists.view(-1, 10, 4)
mean = torch.mean(save_list, 1)
with open('result.txt', 'w+') as out:
    for i, v in enumerate(mean):
        out.write('%.4f %.4f %.4f %.4f\n' % (v[0], v[1], v[2], v[3]))
'''
绘制拟合函数
'''
fig, ax = plt.subplots(1, 3, figsize=(9, 3), sharey=False)
all_list = lists
lists = lists.view(-1, 10, 4)
avg = torch.mean(lists, 1)

# print(avg)
# print(mean)
# print(mean.size())

# z方向偏移
# result_z = mean[1:, 2] - mean[:-1, 2]
# result_x = mean[1:, 0] - mean[:-1, 0]
# result_y = mean[1:, 1] - mean[:-1, 1]
ap = 0
result_x = []
result_y = []
result_z = []
for v in all_list[10:]:
    pnt = ap//10
    ap += 1
    result_x.append(v[0]-avg[pnt, 0])
    result_y.append(v[1]-avg[pnt, 1])
    result_z.append(50-(v[2]-avg[pnt, 2]))
result_x = torch.tensor(result_x)
result_y = torch.tensor(result_y)
result_z = torch.tensor(result_z)
print(result_z.size())
label_row_x = all_list[10:, 0]
label_row_y = all_list[10:, 1]
label_row_z = all_list[10:, 2]
# print(label_row.size())
ax[0].scatter(label_row_x, result_x, label="x-偏移")
ax[1].scatter(label_row_y, result_y, label="y-偏移")
ax[2].scatter(label_row_z, result_z, label="z-偏移")
# ax[0].set(ylim=(0, 10))
# ax[1].set(ylim=(0, 10))
# ax[2].set(ylim=(0, 50))

plt.show()
# 定义x、y散点坐标
x = label_row_z
print('x is :\n', label_row_z)
y = result_z
print('y is :\n', y)
# 用3次多项式拟合
f1 = np.polyfit(x, y, 3)
print('f1 is :\n', f1)

p1 = np.poly1d(f1)
print('p1 is :\n', p1)

#也可使用yvals=np.polyval(f1, x)
yvals = p1(x)  # 拟合y值
print('yvals is :\n', yvals)
# 绘图
plot1 = plt.plot(x, y, 's', label='original values')
plot2 = plt.plot(x, yvals, 'r', label='polyfit values')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc=4)  # 指定legend的位置右下角
plt.title('polyfitting')
plt.show()

z = avg[:, 2]
fix_list = []
for v in z[1:]:
    delt_h = -1.413e-10*v*v*v + 1.573e-06*v*v - 0.005153*v + 6.595
    temp = delt_h
    fix_list.append(temp)
fix_list = torch.tensor(fix_list)
print("修正后的误差", z[1:] - z[:-1] + fix_list)
