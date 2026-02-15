import numpy as np
import matplotlib.pyplot as plt

# python-control library (install via: pip install control)
import control as ctl

# Physical parameters for a single robotic joint
J = 0.01     # kg m^2
b = 0.1      # N m s/rad

# PI controller parameters (chosen to satisfy basic specs)
Kp = 20.0
Ki = 50.0

# Define transfer functions: G(s) and C(s)
s = ctl.TransferFunction.s
G = 1 / (J * s**2 + b * s)
C = Kp + Ki / s

L = C * G                 # loop transfer
S = 1 / (1 + L)           # sensitivity
T = L / (1 + L)           # complementary sensitivity

# Disturbance input at plant input: G_d(s) = S(s) G(s)
Gd_in = ctl.minreal(S * G, verbose=False)

# Frequency grid and magnitude of S and Gd_in
w = np.logspace(-1, 2, 500)   # rad/s
magS, phaseS, _ = ctl.freqresp(S, w)
magGd, phaseGd, _ = ctl.freqresp(Gd_in, w)

magS_db = 20 * np.log10(np.abs(magS).flatten())
magGd_db = 20 * np.log10(np.abs(magGd).flatten())

plt.figure(figsize=(8, 5))
plt.semilogx(w, magS_db, label="|S(jw)|")
plt.semilogx(w, magGd_db, label="|G_d_in(jw)| = |S(jw) G(jw)|")
plt.axhline(-40, color="k", linestyle="--", label="-40 dB target")
plt.xlabel("Frequency w [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.title("Sensitivity and Disturbance Transfer for Robotic Joint")
plt.grid(True, which="both")
plt.legend()
plt.tight_layout()
plt.show()

# Time-domain simulation: step load disturbance
t = np.linspace(0, 5, 1000)
D0 = 1.0  # step load torque [N m]
d_step = D0 * np.ones_like(t)

# Closed-loop from load torque to position: Gd_in(s)
t_out, theta_out = ctl.forced_response(Gd_in, T=t, U=d_step)

print("Approx. steady-state position error [rad]:", theta_out[-1])
