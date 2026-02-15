
import numpy as np

A = np.array([[0.0, 1.0],
              [0.0, -0.5]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 0.0]])
K = np.array([[3.0, 2.0]])      # state-feedback gain
L = np.array([[5.0],
              [6.0]])           # observer gain

Ts = 0.001
steps = 5000

x = np.array([[0.5],
              [0.0]])           # true initial state
x_hat = np.zeros((2, 1))        # initial estimate
rng = np.random.default_rng(0)

for k in range(steps):
    t = k * Ts

    # measurement with additive Gaussian noise
    y = C @ x + 0.01 * rng.standard_normal((1, 1))

    # control uses estimated state
    u = -K @ x_hat

    # true and estimated dynamics (Euler discretization)
    x_dot = A @ x + B @ u
    x_hat_dot = A @ x_hat + B @ u + L @ (y - C @ x_hat)

    x += Ts * x_dot
    x_hat += Ts * x_hat_dot

    if k % 1000 == 0:
        print(f"t={t:5.3f}  q={x[0,0]: .3f}  q_hat={x_hat[0,0]: .3f}")
