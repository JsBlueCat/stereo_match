import torch
from matplotlib import pyplot as plt
import numpy as np

with open('pingjun.txt') as file:
    lists = [s.split(' ') for s in file.read().split('\n')][:-1]
    for i, v in enumerate(lists):
        lists[i] = [float(st) for st in v]
    lists = torch.Tensor(lists)

with open('real_points.txt') as file:
    real_points = [s.split(' ') for s in file.read().split('\n')][:-1]
    for i, v in enumerate(real_points):
        real_points[i] = [float(st) for st in v]
    real_points = torch.Tensor(real_points)

# print(lists)
# print(real_points)

predict_z = lists[:,2][::30]
real_z =  real_points[:,2][::30]

x  = predict_z.numpy()
y  = real_z.numpy()
print(x,y)

# 用3次多项式拟合
f_z = np.polyfit(predict_z.numpy(), real_z.numpy(), 4)

with open('rectify_z_param.txt','a+') as rec_file:
    for param in f_z:
        rec_file.write('%.30e ' % param)
    rec_file.write('\n')

p_z = np.poly1d(f_z)
print('p_z is :\n', p_z)

# 绘图
plot1 = plt.plot(predict_z.numpy(),  real_z.numpy(), '*', label='original values')
plot2 = plt.plot(predict_z.numpy(), p_z(predict_z.numpy()), 'r', label='polyfit values')

rectified_vals = p_z(lists[:,2].numpy())
print('rectified vals is :\n', rectified_vals)

plt.xlabel('predict_z')
plt.ylabel('real_z')
plt.legend(loc=4)  # 指定legend的位置右下角
plt.title('polyfitting')
plt.show()


torch_rectified_val = torch.from_numpy(rectified_vals)
lists[:,2] = torch_rectified_val
print(lists)

with open('perdict_rectfy.txt', 'w+') as out:
    for p in lists:
        out.write('%.4f %.4f %.4f\n' % (p[0], p[1], p[2]))

