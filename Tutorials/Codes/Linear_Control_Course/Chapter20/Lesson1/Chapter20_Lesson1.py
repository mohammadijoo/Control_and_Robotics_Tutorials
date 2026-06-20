import numpy as np

# Second-order plant parameters (e.g. robot joint)
zeta = 0.3
omega_n = 4.0
k_plant = 1.0  # input gain

def plant_step(x, u):
    """
    x = [position; velocity]
    x_dot = [v; -2 zeta omega_n v - omega_n^2 y + k_plant u]
    """
    y = x[0]
    v = x[1]
    dy = v
    dv = -2.0 * zeta * omega_n * v - (omega_n ** 2) * y + k_plant * u
    return np.array([dy, dv])

# 1-DOF proportional controller
Kp_1dof = 20.0

# 2-DOF controller: same feedback gain, plus first-order prefilter
Ky_2dof = 20.0
Kr_2dof = 20.0
T_f = 0.2  # reference filter time constant

dt = 1e-3
T_end = 2.0
steps = int(T_end / dt)

r = 1.0  # step reference

x_1 = np.zeros(2)  # state for 1-DOF
x_2 = np.zeros(2)  # state for 2-DOF
r_f = 0.0          # filtered reference for 2-DOF

y_hist_1 = np.zeros(steps)
y_hist_2 = np.zeros(steps)

for k_step in range(steps):
    # Current outputs
    y1 = x_1[0]
    y2 = x_2[0]

    # 1-DOF control: u = Kp (r - y)
    u1 = Kp_1dof * (r - y1)

    # 2-DOF control:
    # reference prefilter r_f_dot = (r - r_f) / T_f
    r_f += dt * (r - r_f) / T_f
    # u = Kr r_f - Ky y
    u2 = Kr_2dof * r_f - Ky_2dof * y2

    # Integrate plant dynamics for both loops
    x_1 = x_1 + dt * plant_step(x_1, u1)
    x_2 = x_2 + dt * plant_step(x_2, u2)

    y_hist_1[k_step] = x_1[0]
    y_hist_2[k_step] = x_2[0]

# Compare final values (both should approach 1.0)
print("Final 1-DOF position:", y_hist_1[-1])
print("Final 2-DOF position:", y_hist_2[-1])

# For visualization, plot using matplotlib (not shown here):
# import matplotlib.pyplot as plt
# t = np.arange(steps) * dt
# plt.plot(t, y_hist_1, label="1-DOF")
# plt.plot(t, y_hist_2, label="2-DOF")
# plt.legend(); plt.xlabel("time"); plt.ylabel("y"); plt.show()
