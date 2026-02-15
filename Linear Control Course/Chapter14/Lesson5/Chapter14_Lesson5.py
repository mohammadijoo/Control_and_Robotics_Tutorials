import numpy as np
import control as ct  # python-control
# For robotics models you can use:
# from roboticstoolbox import DHRobot, RevoluteDH, etc.

# Design parameters
wn = 10.0      # natural frequency [rad/s]
zeta = 0.4     # damping ratio
K = 2.0        # proportional gain

# Plant G(s) = wn^2 / (s^2 + 2*zeta*wn s + wn^2)
numG = [wn**2]
denG = [1.0, 2.0*zeta*wn, wn**2]
G = ct.TransferFunction(numG, denG)

# Controller C(s) = K
C = ct.TransferFunction([K], [1.0])

# Loop transfer function L(s) and closed-loop T(s)
L = C * G
T = ct.feedback(L, 1)  # unity feedback

# Bode plot of L(s) and computation of margins
w = np.logspace(-1, 2, 500)  # frequency grid
mag, phase, omega = ct.bode(L, w, Plot=False)

gm, pm, w_pc, w_gc = ct.margin(L)
print(f"Gain margin (dB): {20*np.log10(gm):.2f}")
print(f"Phase margin (deg): {pm:.2f}")
print(f"Gain crossover freq w_gc: {w_gc:.2f} rad/s")
print(f"Phase crossover freq w_pc: {w_pc:.2f} rad/s")

# Closed-loop bandwidth from T(s) magnitude
magT, phaseT, omegaT = ct.bode(T, w, Plot=False)
# Find -3 dB bandwidth (magnitude drop by factor 1/sqrt(2))
mag0 = magT[0]
idx_bw = np.where(magT < mag0/np.sqrt(2))[0]
if idx_bw.size > 0:
    wb = omegaT[idx_bw[0]]
    print(f"Approximate closed-loop bandwidth wb: {wb:.2f} rad/s")
    ts_approx = 4.0 / wb
    print(f"Approximate settling time ts ~ 4/wb: {ts_approx:.2f} s")

# For visualization (in a Jupyter notebook) you can call:
# ct.bode_plot(L)
# ct.bode_plot(T)
