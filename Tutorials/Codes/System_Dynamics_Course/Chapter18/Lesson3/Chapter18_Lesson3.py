# Chapter18_Lesson3.py
# Hamiltonian and Port-Hamiltonian simulation (mass-spring-damper with input port)
# x = [q, p], H = 0.5*k*q^2 + p^2/(2*m)

import numpy as np
import matplotlib.pyplot as plt

m = 1.5      # mass
k = 12.0     # spring constant
d = 0.8      # viscous damping coefficient
omega = 1.4  # input frequency

J = np.array([[0.0, 1.0],
              [-1.0, 0.0]])
R = np.array([[0.0, 0.0],
              [0.0, d]])
g = np.array([[0.0],
              [1.0]])

def grad_H(x):
    q, p = x
    return np.array([k * q, p / m])

def H(x):
    q, p = x
    return 0.5 * k * q * q + 0.5 * p * p / m

def u(t):
    return np.sin(omega * t)

def y_output(x):
    # y = g^T * gradH = p/m
    return x[1] / m

def dynamics(t, x):
    gh = grad_H(x)
    dx = (J - R) @ gh + (g[:, 0] * u(t))
    return dx

def rk4_step(t, x, h):
    k1 = dynamics(t, x)
    k2 = dynamics(t + 0.5 * h, x + 0.5 * h * k1)
    k3 = dynamics(t + 0.5 * h, x + 0.5 * h * k2)
    k4 = dynamics(t + h, x + h * k3)
    return x + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

T = 20.0
h = 0.002
N = int(T / h) + 1
t_grid = np.linspace(0.0, T, N)

x = np.array([0.15, 0.0])  # initial [q, p]
X = np.zeros((N, 2))
Hvals = np.zeros(N)
yvals = np.zeros(N)
uvals = np.zeros(N)
diss = np.zeros(N)  # gradH^T R gradH

for i, t in enumerate(t_grid):
    X[i, :] = x
    gh = grad_H(x)
    Hvals[i] = H(x)
    yvals[i] = y_output(x)
    uvals[i] = u(t)
    diss[i] = gh @ (R @ gh)
    if i < N - 1:
        x = rk4_step(t, x, h)

# Numerical check of pH power balance:
# dH/dt = -diss + y*u
supply = yvals * uvals
rhs = -diss + supply
energy_change = Hvals[-1] - Hvals[0]
rhs_integral = np.trapz(rhs, t_grid)
residual = energy_change - rhs_integral

print("Final state [q, p] =", X[-1])
print("Initial energy =", Hvals[0])
print("Final energy   =", Hvals[-1])
print("Energy balance residual (should be small) =", residual)

# Save a CSV for cross-language comparison
data = np.column_stack([t_grid, X[:, 0], X[:, 1], Hvals, uvals, yvals, diss, supply])
header = "t,q,p,H,u,y,dissipation,supply"
np.savetxt("Chapter18_Lesson3_python_output.csv", data, delimiter=",", header=header, comments="")

# Plots
plt.figure(figsize=(10, 4))
plt.plot(t_grid, X[:, 0], label="q(t)")
plt.plot(t_grid, X[:, 1], label="p(t)")
plt.xlabel("Time [s]")
plt.ylabel("States")
plt.title("Port-Hamiltonian states")
plt.legend()
plt.tight_layout()

plt.figure(figsize=(10, 4))
plt.plot(t_grid, Hvals, label="H(t)")
plt.plot(t_grid, np.cumsum(rhs) * h + Hvals[0], label="H(0)+int(-diss+y*u)dt", linestyle="--")
plt.xlabel("Time [s]")
plt.ylabel("Energy")
plt.title("Energy balance verification")
plt.legend()
plt.tight_layout()

plt.show()
