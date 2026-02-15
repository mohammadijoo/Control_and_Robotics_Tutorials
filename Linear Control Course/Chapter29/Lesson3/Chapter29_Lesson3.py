import numpy as np
import matplotlib.pyplot as plt
import control as ctrl  # python-control

# Identified thermal parameters
tau_th = 300.0   # seconds
K_th   = 5.0     # K per unit input

# First-order plant G_th(s) = K_th / (tau_th s + 1)
G_th = ctrl.tf([K_th], [tau_th, 1.0])

# Desired closed-loop specs
zeta = 0.7
Ts_des = 1200.0  # desired 2% settling time in seconds

# Settling time approximation: Ts ≈ 4 / (zeta * omega_n)
omega_n = 4.0 / (zeta * Ts_des)

# Compute PI gains from matching
Kp = (2.0 * zeta * omega_n * tau_th - 1.0) / K_th
Ki = (omega_n**2 * tau_th) / K_th

print(f"Kp = {Kp:.4f}, Ki = {Ki:.4f}")

# PI controller C(s) = Kp + Ki/s = (Kp*s + Ki)/s
C = ctrl.tf([Kp, Ki], [1.0, 0.0])

# Closed-loop (unity feedback)
T_cl = ctrl.feedback(C * G_th, 1.0)

# Step response: 1 K step in reference
t = np.linspace(0, 4000, 1000)
t, y = ctrl.step_response(T_cl, T=t)

plt.figure()
plt.plot(t, y, label="T(t)")
plt.xlabel("Time [s]")
plt.ylabel("Temperature change [K]")
plt.title("Closed-loop step response of thermal system with PI control")
plt.grid(True)
plt.legend()
plt.show()

# Disturbance rejection: treat disturbance input via additional transfer function if needed.
