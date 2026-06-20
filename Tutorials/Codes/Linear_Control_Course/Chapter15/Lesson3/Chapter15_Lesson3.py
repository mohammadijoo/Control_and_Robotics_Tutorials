import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control

# Example plant: G(s) = 1 / (s (s + 1) (s + 2))
num = [1.0]
den = [1.0, 3.0, 2.0, 0.0]
G = ct.TransferFunction(num, den)

K = 2.0  # nominal proportional gain
L = K * G

# Nyquist plot
omega = np.logspace(-2, 2, 1000)
real, imag, freq = ct.nyquist_plot(L, omega, Plot=False)

plt.figure()
plt.plot(real, imag)
plt.plot(real, -imag)  # mirror for full Nyquist if needed
plt.scatter([-1.0], [0.0], marker="x")  # critical point -1
plt.axhline(0, linewidth=0.5)
plt.axvline(0, linewidth=0.5)
plt.xlabel("Re(L(j*omega))")
plt.ylabel("Im(L(j*omega))")
plt.title("Nyquist plot for L(s) with K = %.2f" % K)
plt.gca().set_aspect("equal", "box")
plt.grid(True)

# Approximate minimum distance to -1 for relative stability
z = real + 1j * imag
d = np.abs(1.0 + z)
d_min = np.min(d)
omega_min = freq[np.argmin(d)]
print("Approximate d_min =", d_min, "at omega =", omega_min)

# Approximate gain margin by scaling K until Nyquist passes -1
# (brute-force search over a grid of gains)
K_grid = np.linspace(0.1, 10.0, 200)
gm_est = None
for Kg in K_grid:
    Lg = Kg * G
    rg, ig, fg = ct.nyquist_plot(Lg, omega, Plot=False)
    zg = rg + 1j * ig
    dg = np.min(np.abs(1.0 + zg))
    if dg < 1e-2:  # "close enough" to -1
        gm_est = Kg / K
        break

print("Estimated gain margin (Nyquist-based) GM ~", gm_est)

plt.show()
