import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Double-integrator plant: G(s) = 1 / s^2
s = ctrl.TransferFunction.s
G = 1 / s**2

# Design specs
Mp_max = 0.10     # 10% overshoot (approx)
zeta = 0.6        # typical damping for ~10% overshoot
ts_des = 1.0      # desired 2% settling time [s]

# Natural frequency from ts ≈ 4/(zeta*omega_n)
omega_n = 4.0 / (zeta * ts_des)   # ≈ 6.666...

# PD gains for canonical second-order closed loop
Kd = 2.0 * zeta * omega_n         # ≈ 8.0
Kp = omega_n**2                   # ≈ 44.4

C_pd = Kd * s + Kp

# Closed-loop from reference to output without prefilter
T0 = ctrl.feedback(C_pd * G, 1)

# Add a first-order prefilter F(s) to smooth the command
omega_f = 0.7 * omega_n           # slightly below closed-loop bandwidth
F = omega_f / (s + omega_f)

# Prefiltered tracking transfer: T = F * T0
T = ctrl.minreal(F * T0)

# Step responses
t = np.linspace(0, 3, 1000)
t1, y_no_pref = ctrl.step_response(T0, T=t)
t2, y_pref = ctrl.step_response(T, T=t)

plt.figure()
plt.plot(t1, y_no_pref, label="Without prefilter")
plt.plot(t2, y_pref, label="With prefilter F(s)")
plt.axhline(1.0, color="k", linestyle="--", linewidth=0.8)
plt.xlabel("Time [s]")
plt.ylabel("Output position")
plt.grid(True)
plt.legend()
plt.title("Set-point tracking for robot joint (double integrator + PD)")
plt.show()
