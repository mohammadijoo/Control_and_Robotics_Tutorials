import numpy as np
import control as ct  # python-control library
import matplotlib.pyplot as plt

# Joint-like plant: P(s) = 1 / (s (s + 1))
s = ct.TransferFunction.s
P = 1 / (s * (s + 1))

# Simple PD controller (chosen arbitrarily for demonstration)
Kp = 40.0
Kd = 2.0
C = Kp + Kd * s

# Nominal open loop without explicit delay term
L0 = C * P

# Classical margins (gain margin, phase margin, crossover frequencies)
gm, pm_deg, wgc, wpc = ct.margin(L0)
print("Phase margin [deg]:", pm_deg)
print("Gain crossover [rad/s]:", wgc)

# Delay margin from phase margin and crossover frequency
pm_rad = pm_deg * np.pi / 180.0
D_max = pm_rad / wgc
print("Approximate delay margin [s]:", D_max)

# Now include an explicit delay via Padé approximation
# Example nominal delay for communication/computation:
L_nom = 0.03  # 30 ms
num_d, den_d = ct.pade(L_nom, 1)  # first-order Padé
D_pade = ct.TransferFunction(num_d, den_d)

L_with_delay = C * P * D_pade
T_with_delay = ct.feedback(L_with_delay, 1)

# Compare step responses for different additional delays
delays = [0.0, 0.02, 0.05, 0.08]  # added to nominal delay
t = np.linspace(0, 5, 1000)

plt.figure()
for d_add in delays:
    # total delay approximated as Padé of (L_nom + d_add)
    num_d2, den_d2 = ct.pade(L_nom + d_add, 1)
    D_tot = ct.TransferFunction(num_d2, den_d2)
    Ld = C * P * D_tot
    T_d = ct.feedback(Ld, 1)
    t_out, y_out = ct.step_response(T_d, T=t)
    label = f"delay = {(L_nom + d_add):.3f} s"
    plt.plot(t_out, y_out, label=label)

plt.xlabel("Time [s]")
plt.ylabel("Joint angle (normalized)")
plt.title("Step response vs. time delay (Padé approximation)")
plt.legend()
plt.grid(True)
plt.show()

# Note: In a robotics stack (e.g. ROS), P and C may come
# from identified models of a DC motor + load, and L_nom
# corresponds to sensing and communication delays.
