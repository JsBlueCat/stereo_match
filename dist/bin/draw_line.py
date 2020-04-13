import torch
from matplotlib import pyplot as plt
import numpy as np

with open('pingjun.txt') as file:
    lists = [s.split(' ') for s in file.read().split('\n')]
    for i, v in enumerate(lists):
        lists[i] = [float(st) for st in v]
    lists = torch.Tensor(lists)
# labels = [(2745 + 602 * i) for i in range(15)]
# labels = -8 - lists[:,1]
labels = torch.Tensor([2304,2932,3471,4006,4532.5,5028,5552,6052,6458.5,6989,7547.7,7888,8392.5,8889.5,9324,9775.7,10441.2,10980.7]) - lists[:, 2]
# labels = lists[:,1].numpy() - 51.5

# x  = lists[:,1].numpy() - 51.5
x = lists[:, 2].numpy()
print('x is :\n', x)
y = np.array(labels)
print('y is :\n', y)
# 用3次多项式拟合
f1 = np.polyfit(x, y, 8)
print('f1 is :\n', f1)
with open('rectify.txt','a+') as rec_file:
    for param in f1:
        rec_file.write('%.30e ' % param)
    rec_file.write('\n')
p1 = np.poly1d(f1)
print('p1 is :\n', p1)
#也可使用yvals=np.polyval(f1, x)
yvals = p1(x)  # 拟合y值
print('yvals is :\n', yvals)

# 绘图
plot1 = plt.plot(x, y, '*', label='original values')
plot2 = plt.plot(x, yvals, 'r', label='polyfit values')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc=4)  # 指定legend的位置右下角
plt.title('polyfitting')
plt.show()
