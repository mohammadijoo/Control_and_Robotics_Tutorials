
import numpy as np

# Simulation parameters
dt = 0.001
T  = 5.0
N  = int(T / dt)

# Robot and environment
M_r = 5.0   # robot reflected mass
D_r = 2.0   # robot structural damping

K_e = 500.0  # environment stiffness
D_e = 10.0   # environment damping

# Desired impedance parameters
M_d = 2.0
D_d = 30.0
K_d = 200.0

# Reference motion: constant position
x_d = 0.05  # 5 cm into the environment

# Data arrays
t  = np.linspace(0.0, T, N)
x  = np.zeros(N)
v  = np.zeros(N)
F  = np.zeros(N)     # interaction force
x_c = np.zeros(N)    # for admittance
mode = "impedance"   # or "admittance"

def environment_force(xi, vi):
    # xi: end-effector position, vi: velocity
    return K_e * xi + D_e * vi

for k in range(N - 1):
    # Interaction force from environment
    F[k] = environment_force(x[k], v[k])

    if mode == "impedance":
        # Impedance control: compute control force F_u
        e  = x_d - x[k]
        ed = 0.0 - v[k]  # desired velocity = 0
        # Discrete approximation of acceleration error is omitted; we use PD + stiffness
        F_u = K_d * e + D_d * ed

        # Robot dynamics: M_r * a = F_u + F
        a = (F_u + F[k] - D_r * v[k]) / M_r

        # Euler integration
        v[k+1] = v[k] + dt * a
        x[k+1] = x[k] + dt * v[k]

    elif mode == "admittance":
        # Admittance control: map force to virtual position x_c
        if k == 0:
            x_c[k] = 0.0
            v_c = 0.0
        else:
            v_c = (x_c[k] - x_c[k-1]) / dt

        # Admittance dynamics in discrete time (explicit Euler)
        # M_d * a_c + D_d * v_c + K_d * (x_c - x_d) = F
        a_c = (F[k] - D_d * v_c - K_d * (x_c[k] - x_d)) / M_d
        v_c_new = v_c + dt * a_c
        x_c[k+1] = x_c[k] + dt * v_c_new

        # Inner position loop (very stiff)
        # Robot tries to track x_c
        e_pos = x_c[k] - x[k]
        F_u = 1000.0 * e_pos  # large proportional gain on position

        a = (F_u + F[k] - D_r * v[k]) / M_r
        v[k+1] = v[k] + dt * a
        x[k+1] = x[k] + dt * v[k+1]

# After simulation, t, x, F can be plotted using matplotlib
