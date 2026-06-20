import numpy as np
from control.matlab import tf, lsim  # python-control library
import matplotlib.pyplot as plt

# Continuous-time plant: approximate DC motor (position dynamics)
# G(s) = K / (tau s + 1) / s
K = 10.0
tau = 0.05
num = [K]
den = [tau, 1.0, 0.0]  # tau s^2 + s
G = tf(num, den)

# Experiment setup
Ts = 0.001   # sampling period [s]
T_total = 5.0
t = np.arange(0.0, T_total, Ts)

# Frequencies to test (rad/s)
omegas = np.array([1.0, 5.0, 10.0, 20.0])
U0 = 0.1  # input amplitude

def sine_input(omega, t, U0):
    return U0 * np.sin(omega * t)

def estimate_G_at_frequency(u, y, Ts, omega):
    """
    Estimate G(j*omega) from time data using DFT at the corresponding frequency bin.
    For simplicity we assume the record has an integer number of periods.
    """
    N = len(u)
    # Compute DFT via FFT
    U = np.fft.fft(u)
    Y = np.fft.fft(y)
    # Frequency bins
    freqs = 2.0 * np.pi * np.fft.fftfreq(N, d=Ts)  # angular frequencies
    # Find the index closest to omega
    idx = np.argmin(np.abs(freqs - omega))
    Ghat = Y[idx] / U[idx]
    return Ghat, freqs[idx]

Ghat_list = []

for omega in omegas:
    u = sine_input(omega, t, U0)
    # continuous-time simulation, then sampled
    tout, y, _ = lsim(G, U=u, T=t)
    # discard transient: keep last half of the record
    half = len(u) // 2
    u_ss = u[half:]
    y_ss = y[half:]
    Ghat, omega_eff = estimate_G_at_frequency(u_ss, y_ss, Ts, omega)
    Ghat_list.append(Ghat)
    print(f"omega target = {omega:.2f}, omega_eff = {omega_eff:.2f}, "
          f"|Ghat| = {np.abs(Ghat):.3f}, angle = {np.angle(Ghat):.3f} rad")

# Optional: compare with theoretical G(j*omega)
from control.matlab import bode

# bode returns magnitude and phase on a grid; we evaluate at omegas
mag, phase, _ = bode(G, omegas, Plot=False)
for i, omega in enumerate(omegas):
    print(f"Theoretical |G(j{omega:.2f})| = {mag[i][0]:.3f}, phase = {phase[i][0]:.3f} rad")

# Plot estimated vs theoretical magnitude
plt.figure()
plt.loglog(omegas, [np.abs(g) for g in Ghat_list], "o-", label="Estimated")
plt.loglog(omegas, [m[0] for m in mag], "x--", label="Theoretical")
plt.xlabel("omega [rad/s]")
plt.ylabel("|G(j omega)|")
plt.legend()
plt.grid(True)
plt.show()
