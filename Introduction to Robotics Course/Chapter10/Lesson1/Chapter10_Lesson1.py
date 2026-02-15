import numpy as np

A = np.array([[1.0, 0.01],
              [0.0, 1.0]])
B = np.array([[0.0],
              [0.01]])
K = np.array([[2.0, 0.5]])

x = np.array([[0.1],
              [0.0]])

def step(x):
    u = -K @ x
    return A @ x + B @ u

for k in range(1000):
    x = step(x)
print(x.ravel())
