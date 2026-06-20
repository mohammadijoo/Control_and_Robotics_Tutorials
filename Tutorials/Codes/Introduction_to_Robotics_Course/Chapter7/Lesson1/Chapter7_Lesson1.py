import numpy as np

# 1D robot: x=[p, v]^T
Ts = 0.1
A = np.array([[1, Ts],
              [0, 1]])
B = np.array([[0.5*Ts**2],
              [Ts]])

# true state
x = np.array([[0.0],
              [1.0]])

# proprio velocity sensor bias
b = 0.05
sigma_p = 0.02
sigma_e = 0.05

p_hat_from_proprio = 0.0

N = 200
p_true_hist, p_hat_hist, p_extero_hist = [], [], []

for k in range(N):
    u = np.array([[0.0]])  # constant velocity case
    w = np.zeros((2,1))
    x = A @ x + B @ u + w

    # proprio sensor: velocity with bias
    y_p = x[1,0] + b + np.random.randn()*sigma_p
    p_hat_from_proprio += Ts * y_p

    # extero sensor: position
    y_e = x[0,0] + np.random.randn()*sigma_e

    p_true_hist.append(x[0,0])
    p_hat_hist.append(p_hat_from_proprio)
    p_extero_hist.append(y_e)

print("Final true position:", p_true_hist[-1])
print("Final proprio-integrated estimate:", p_hat_hist[-1])
print("Final extero measurement:", p_extero_hist[-1])
