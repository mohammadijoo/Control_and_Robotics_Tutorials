import numpy as np
import matplotlib.pyplot as plt

# Loop transfer function L(s) = K / (s (s + 1))
def loop_L(jw, K):
    s = 1j * jw
    P = 1.0 / (s * (s + 1.0))
    C = K
    return C * P

def G_ref(jw, K):
    """Closed-loop transfer from reference to output: Y/R = L/(1+L)."""
    L = loop_L(jw, K)
    return L / (1.0 + L)

def G_err(jw, K):
    """Closed-loop transfer from reference to error: E/R = 1/(1+L)."""
    L = loop_L(jw, K)
    return 1.0 / (1.0 + L)

# Design parameter: try a few candidate gains
K = 50.0

w_vals = np.logspace(-2, 2, 400)  # 0.01 to 100 rad/s
err_mag = np.array([np.abs(G_err(w, K)) for w in w_vals])
ref_mag = np.array([np.abs(G_ref(w, K)) for w in w_vals])

print("Approx. error magnitude at w = 1 rad/s:",
      np.abs(G_err(1.0, K)))

# Plot magnitude of error and tracking transfer functions
plt.figure()
plt.loglog(w_vals, err_mag, label="|E(jw)/R(jw)|")
plt.loglog(w_vals, np.abs(1.0 - ref_mag), linestyle="--",
           label="|1 - Y(jw)/R(jw)|")
plt.axvline(1.0, linestyle=":", label="w_r = 1 rad/s")
plt.axhline(0.02, linestyle=":", label="delta = 0.02")
plt.xlabel("Frequency w [rad/s]")
plt.ylabel("Amplitude ratio")
plt.legend()
plt.grid(True, which="both")
plt.title("Low-frequency tracking error vs loop gain K")

plt.show()
