import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = Axes3D(fig)
x = np.linspace(0,10)
y = x
X, Y = np.meshgrid(x, y)
Z = pow(X, 2) + pow(Y, 2)
plt.title('demo')
plt.xlabel('X')
plt.ylabel('Y')
ax.scatter(X, Y, Z, 'b-', label='X^2+Y^2')
plt.legend()
plt.show()