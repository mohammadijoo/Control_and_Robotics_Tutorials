import numpy as np
import matplotlib.pyplot as plt

# Core control library (python-control)
import control as ct

# Plant: first-order model of a robot joint or motor: P(s) = K_g / (T s + 1)
K_g = 2.0
T = 0.5
P = ct.tf([K_g], [T, 1.0])

# Design target: ramp steady-state error e_ss_ramp <= 0.05
# We know e_ss_ramp = 1 / (K_I * K_g), so K_I * K_g >= 20 => K_I >= 10.
K_I = 10.0  # Choose just at the bound
C_int = ct.tf([K_I], [1.0, 0.0])  # C(s) = K_I / s

L = C_int * P
T_cl = ct.feedback(L, 1)  # unity-feedback closed-loop transfer function

# Frequency response to inspect low-frequency shape
w = np.logspace(-2, 2, 400)
mag, phase, omega = ct.bode(L, w, Plot=False)

plt.figure()
plt.subplot(2, 1, 1)
plt.semilogx(omega, 20 * np.log10(mag))
plt.ylabel("Magnitude (dB)")
plt.grid(True, which="both")

plt.subplot(2, 1, 2)
plt.semilogx(omega, phase * 180.0 / np.pi)
plt.ylabel("Phase (deg)")
plt.xlabel("Frequency (rad/s)")
plt.grid(True, which="both")
plt.suptitle("Loop transfer function L(j omega) with integral action")
plt.tight_layout()
plt.show()

# Time-domain simulation for step and ramp inputs
t = np.linspace(0.0, 10.0, 1000)
t_step, y_step = ct.step_response(T_cl, T=t)

# Ramp response: simulate by feeding an equivalent input through closed loop
# We approximate r(t) = t by integrating a unit step (command shaping).
# Alternatively, directly simulate via forced_response with r(t) = t.
r_ramp = t
t_ramp, y_ramp, x_ramp = ct.forced_response(T_cl, T=t, U=r_ramp)

e_ramp = r_ramp - y_ramp
e_ss_ramp_est = e_ramp[-1]
print("Estimated ramp steady-state error:", e_ss_ramp_est)

plt.figure()
plt.plot(t_ramp, r_ramp, label="r(t) = ramp")
plt.plot(t_ramp, y_ramp, label="y(t)")
plt.plot(t_ramp, e_ramp, label="e(t)")
plt.xlabel("Time (s)")
plt.ylabel("Signals")
plt.grid(True)
plt.legend()
plt.title("Ramp tracking with integral action")
plt.show()
