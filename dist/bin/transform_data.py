import torch
BASH_LEN_X = 299.3
BASH_LEN_Y = 84
BASH_LEN_Z = 0
points = []
for lr in range(2):
    base_x = pow(-1, lr) * BASH_LEN_X
    for col in range(2):
        for row in range(8):
            points.append((pow(-1, col+1) * 15 + base_x,
                           (7-row) * 30 + BASH_LEN_Y, BASH_LEN_Z + lr * 102))
points = torch.Tensor(points)
print(points)
