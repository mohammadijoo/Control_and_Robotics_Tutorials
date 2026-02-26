import numpy as np
import control as ctl
import matplotlib.pyplot as plt

# Motor parameters
R = 2.0
L = 0.5
J = 0.02
b = 0.002
Kt = 0.1
Ke = 0.1

# Effective first-order speed model parameters
Km = Kt / (b * R + Kt * Ke)
Tm = J * R / (b * R + Kt * Ke)

print("Km =", Km, "Tm =", Tm)

# First-order speed plant G_omega(s) = Km / (Tm s + 1)
G_omega = ctl.tf([Km], [Tm, 1.0])

# Desired closed-loop specs for speed
zeta = 0.8
omega_n = 20.0  # rad/s

Kp = (2 * zeta * omega_n * Tm - 1.0) / Km
Ki = (Tm * omega_n**2) / Km

print("Kp =", Kp, "Ki =", Ki)

# PI controller C(s) = Kp + Ki / s
C_speed = ctl.tf([Kp, Ki], [1.0, 0.0])

# Closed-loop speed from speed reference to speed
T_speed = ctl.feedback(C_speed * G_omega, 1)

# Position loop: integrate speed
G_pos_eff = ctl.series(ctl.tf([1.0], [1.0, 0.0]), T_speed)

# Choose outer loop natural frequency lower than inner
omega_n_theta = 5.0
zeta_theta = 0.9
tau_v = 1.0 / omega_n  # approximate from inner loop bandwidth
Kp_theta = tau_v * omega_n_theta**2

C_pos = ctl.tf([Kp_theta], [1.0])

T_pos = ctl.feedback(C_pos * G_pos_eff, 1)

# Time vector and responses
t_speed, y_speed = ctl.step_response(T_speed, T=np.linspace(0, 1.0, 1000))
t_pos, y_pos = ctl.step_response(T_pos, T=np.linspace(0, 2.0, 1000))

plt.figure()
plt.plot(t_speed, y_speed)
plt.xlabel("Time [s]")
plt.ylabel("Speed [rad/s]")
plt.title("Closed-loop speed response (unit step)")
plt.grid(True)

plt.figure()
plt.plot(t_pos, y_pos)
plt.xlabel("Time [s]")
plt.ylabel("Position [rad]")
plt.title("Closed-loop position response (unit step)")
plt.grid(True)

plt.show()
