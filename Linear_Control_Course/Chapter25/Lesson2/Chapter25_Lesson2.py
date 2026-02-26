import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Inner plant parameters (example DC motor current loop)
L = 2e-3     # H
R = 0.5      # Ohm
Ku = 1.0     # converter gain (V-to-current scaling)

# Desired inner closed-loop specs
zeta_i = 0.7
omega_n_i = 500.0  # rad/s, gives very fast inner response

# PI design from the analytical formulas
kp = (2.0 * L * zeta_i * omega_n_i - R) / Ku
ki = (L * omega_n_i**2) / Ku

print("Inner PI gains: kp = {:.3f}, ki = {:.3f}".format(kp, ki))

# Transfer function models
s = ct.TransferFunction.s
Gi = Ku / (L * s + R)
Ci = kp + ki / s

Li = Ci * Gi               # inner open-loop
Ti = ct.feedback(Li, 1)    # inner closed-loop complementary sensitivity
Si = 1 - Ti                # inner sensitivity

# Step response of inner loop (current reference step)
t, y = ct.step_response(Ti)
plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("i(t) / i_ref")
plt.title("Inner current loop step response")
plt.grid(True)

# Bode magnitude and margins for inner loop
plt.figure()
ct.bode_plot(Li, dB=True, Hz=False, omega_limits=(10, 1e4))
gm, pm, wcg, wcp = ct.margin(Li)
print("Inner loop margins: GM = {:.2f} dB, PM = {:.1f} deg".format(
    20.0 * np.log10(gm), pm))

plt.show()
