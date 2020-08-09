import torch
from matplotlib import pyplot as plt
import numpy as np

points = []
z_len = [1788, 1788 + 1225, 1788 + 1225 + 1177.2,1788 + 1225 + 1177.2 + 1211.5,1788 + 1225 + 1177.2 + 1211.5 + 1225.1,1788 + 1225 + 1177.2 + 1211.5 + 1225.1 + 1177.5]
for depth in z_len:
    # 中心
    points.append((0, -264, depth))
    # 右边
    for i in range(6):
        points.append(((i+1) * 50, -264, depth))
    # 左边
    for i in range(6):
        points.append(((i+1) * -50, -264, depth))
    # 上面
    for i in range(11):
        points.append((0, -264 + (i+1) * -50, depth))
    # 下面
    for i in range(6):
        points.append((0, -264 + (i+1) * 50, depth))
print(points)


with open('real_points.txt', 'w+') as out:
    for p in points:
        out.write('%.4f %.4f %.4f\n' % (p[0], p[1], p[2]))
