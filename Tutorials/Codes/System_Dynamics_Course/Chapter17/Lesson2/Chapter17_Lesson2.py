# Chapter17_Lesson2.py
# Autocorrelation, Power Spectral Density, and White Noise Models

import numpy as np
import matplotlib.pyplot as plt

def autocorr_biased(x, M):
    x = np.asarray(x, float) - np.mean(x)
    N = len(x)
    return np.array([np.dot(x[:N-m], x[m:]) / N for m in range(M + 1)])

rng = np.random.default_rng(17)
N, sigma, b = 2048, 2.0, 0.6
w = sigma * rng.standard_normal(N)                 # white noise
x = np.r_[w[0], w[1:] + b * w[:-1]]               # colored: x[n]=w[n]+b w[n-1]

M = 80
Rw = autocorr_biased(w, M)
Rx = autocorr_biased(x, M)

# PSD from autocorrelation (Wiener-Khinchin route)
Re = np.r_[Rx, Rx[-2:0:-1]]                       # even extension
Sx = np.real(np.fft.fft(Re))
omega = 2 * np.pi * np.arange(len(Sx)) / len(Sx)

# Theory for x[n]=w[n]+b w[n-1]
Sx_th = sigma**2 * (1 + b**2 + 2 * b * np.cos(omega))

print("R_w[0] (variance estimate) =", Rw[0])
print("R_x[0] (variance estimate) =", Rx[0])
print("Theoretical var(x)        =", sigma**2 * (1 + b**2))

plt.figure()
plt.plot(Rw, label="White")
plt.plot(Rx, label="Colored")
plt.xlabel("Lag m"); plt.ylabel("Autocorrelation"); plt.grid(True); plt.legend()

plt.figure()
half = len(omega) // 2
plt.plot(omega[:half], Sx[:half], label="Estimated PSD")
plt.plot(omega[:half], Sx_th[:half], label="Theory")
plt.xlabel("rad/sample"); plt.ylabel("PSD"); plt.grid(True); plt.legend()
plt.show()
