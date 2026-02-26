import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# 2x2 coupled plant: G(s) = 1/(s+1) * [[1, alpha], [alpha, 1]]
s = ctl.TransferFunction.s

def make_plant(alpha):
    g11 = 1/(s + 1)
    g22 = 1/(s + 1)
    g12 = alpha/(s + 1)
    g21 = alpha/(s + 1)
    G = ctl.TransferFunction([[g11, g12],
                              [g21, g22]])
    return G

def closed_loop(alpha, k):
    G = make_plant(alpha)
    # Decentralized proportional controller: C(s) = k * I
    C = ctl.TransferFunction([[k, 0],
                              [0, k]])
    L = G * C
    # Closed-loop from r to y: T(s) = L (I + L)^(-1)
    I2 = ctl.tf2ss(ctl.TransferFunction([[1, 0],
                                         [0, 1]]))
    T = ctl.feedback(L, I2)  # feedback(L, I) = L (I+L)^(-1)
    return T

k = 2.0
alphas = [0.2, 0.9]
t = np.linspace(0, 10, 1000)

# Step in r1, r2 = 0
r1 = np.ones_like(t)
r2 = np.zeros_like(t)
R = np.vstack((r1, r2))

plt.figure()
for alpha in alphas:
    T = closed_loop(alpha, k)
    tout, y, _ = ctl.forced_response(T, T=t, U=R)
    plt.plot(tout, y[0, :], label=f"y1, alpha={alpha}")
    plt.plot(tout, y[1, :], "--", label=f"y2, alpha={alpha}")

plt.xlabel("Time (s)")
plt.ylabel("Outputs y1, y2")
plt.title("Step in r1 with decentralized control for different coupling alpha")
plt.legend()
plt.grid(True)
plt.show()
