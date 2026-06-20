import numpy as np
import matplotlib.pyplot as plt

# First-order servo model parameters (e.g., simplified robot joint)
K = 10.0     # dimensionless gain
tau = 0.05   # time constant [s]

def magnitude_db_first_order(K, tau, w):
    """
    Compute |G(jw)| in dB for G(s) = K / (tau*s + 1)
    where w is a numpy array of frequencies [rad/s].
    """
    jw_tau = 1j * w * tau
    G_jw = K / (1.0 + jw_tau)
    mag = np.abs(G_jw)
    return 20.0 * np.log10(mag)

# Logarithmically spaced frequencies (0.1 to 1000 rad/s)
w = np.logspace(-1, 3, 500)
mag_db = magnitude_db_first_order(K, tau, w)

# Plot magnitude in dB vs log10(w)
plt.figure()
plt.semilogx(w, mag_db)  # log scale on frequency axis
plt.xlabel("Frequency w [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.title("First-order servo magnitude response (robot joint)")
plt.grid(True, which="both")
plt.show()
