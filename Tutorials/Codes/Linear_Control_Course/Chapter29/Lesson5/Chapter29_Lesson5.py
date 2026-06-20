import numpy as np
import matplotlib.pyplot as plt
import control as ctrl  # python-control

# Plant parameters and specs
T = 0.5
K = 1.0
zeta = 0.7
ts_des = 1.0
wn = 4.0 / (zeta * ts_des)   # desired natural frequency

Kp = 0.5 * wn**2
Td = (zeta * wn - 1.0) / Kp

print(f"Kp = {Kp:.3f}, Td = {Td:.3f}, wn = {wn:.3f}")

# Define transfer functions
s = ctrl.TransferFunction.s
G = 1 / (s * (T * s + 1))     # normalized DC motor position plant
C = Kp * (1 + Td * s)         # PD controller

L = C * G
Tcl = ctrl.feedback(L, 1)     # closed-loop from reference to theta

# Step response
t = np.linspace(0.0, 5.0, 1000)
t_out, y_out = ctrl.step_response(Tcl, t)

plt.figure()
plt.plot(t_out, y_out)
plt.xlabel("Time [s]")
plt.ylabel("Position response theta(t)")
plt.title("Closed-loop step response (Python prototype)")
plt.grid(True)

# Sensitivity and complementary sensitivity
S = ctrl.feedback(1, L)
Tmat = ctrl.feedback(L, 1)

w = np.logspace(-2, 2, 500)
magS, phaseS, wS = ctrl.bode(S, w, Plot=False)
magT, phaseT, wT = ctrl.bode(Tmat, w, Plot=False)

plt.figure()
plt.semilogx(wS, 20 * np.log10(magS), label="S(jw)")
plt.semilogx(wT, 20 * np.log10(magT), label="T(jw)")
plt.xlabel("Frequency [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.title("Sensitivity and complementary sensitivity")
plt.grid(True, which="both")
plt.legend()

plt.show()
