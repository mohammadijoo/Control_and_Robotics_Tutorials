import numpy as np

# Physical parameters
m = 1.0   # mass
c = 0.4   # damping
k = 4.0   # stiffness

# State-space matrices for x = [x1; x2] = [y; y_dot]
A = np.array([[0.0,        1.0],
              [-k / m, -c / m]])
B = np.array([[0.0],
              [1.0 / m]])

def f(x, u):
    """
    Continuous-time state equation:
    x_dot = A x + B u
    x: np.array shape (2,)
    u: scalar
    """
    return (A @ x.reshape(2, 1) + B * u).flatten()

# Simulation settings
t0 = 0.0
tf = 10.0
h = 0.001
N = int((tf - t0) / h)

# Initial conditions: y(0) = 1, y_dot(0) = 0
x = np.array([1.0, 0.0])
u = 0.0  # zero-input to study internal dynamics

ts = np.zeros(N + 1)
xs = np.zeros((N + 1, 2))
ts[0] = t0
xs[0, :] = x

for k_step in range(N):
    x_dot = f(x, u)
    x = x + h * x_dot  # explicit Euler
    ts[k_step + 1] = t0 + (k_step + 1) * h
    xs[k_step + 1, :] = x

# xs[:, 0] is position y(t); xs[:, 1] is velocity y_dot(t)
# Plotting (if matplotlib is available):
# import matplotlib.pyplot as plt
# plt.plot(ts, xs[:, 0])
# plt.xlabel("t")
# plt.ylabel("y(t)")
# plt.show()
      
