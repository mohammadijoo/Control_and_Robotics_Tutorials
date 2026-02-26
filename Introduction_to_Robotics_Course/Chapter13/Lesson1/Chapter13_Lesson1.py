import numpy as np

m, b, k = 1.0, 0.6, 4.0
A = np.array([[0.0, 1.0], [-k/m, -b/m]])
B = np.array([[0.0], [1.0/m]])

def euler_sim(h, T=5.0):
    N = int(T/h)
    x = np.zeros((2, N+1))
    u = lambda t: 1.0  # unit step
    for i in range(N):
        x[:, i+1] = x[:, i] + h*(A@x[:, i] + (B*u(i*h)).ravel())
    t = np.linspace(0, T, N+1)
    return t, x

for h in [0.05, 0.2, 0.5]:
    t, x = euler_sim(h)
    print("h =", h, "final q =", x[0, -1])
      
