import numpy as np
import matplotlib.pyplot as plt

# Closed-loop polynomial: P(s, K) = s^3 + 7 s^2 + 10 s + K
D = np.array([1.0, 7.0, 10.0, 0.0])   # coefficients of D(s)
N = np.array([0.0, 0.0, 0.0, 1.0])    # coefficients of N(s) (unity)

def closed_loop_coeffs(K):
    return D + K * N  # s^3 + 7 s^2 + 10 s + K

# K grid
K_min, K_max = 0.0, 80.0
num_K = 200
K_values = np.linspace(K_min, K_max, num_K)

roots = []

for K in K_values:
    coeffs = closed_loop_coeffs(K)
    r = np.roots(coeffs)
    roots.append(r)

roots = np.array(roots)  # shape (num_K, 3)

# Plot root locus
plt.figure()
for i in range(roots.shape[1]):
    plt.plot(np.real(roots[:, i]), np.imag(roots[:, i]), ".-")

plt.axvline(0.0, linestyle="--")  # imaginary axis
plt.xlabel("Real(s)")
plt.ylabel("Imag(s)")
plt.title("Root locus for G(s) = K / (s (s+2) (s+5))")
plt.grid(True)
plt.show()

# Optional: approximate stability range by checking Re(s) < 0
stable_K = []
for K, r in zip(K_values, roots):
    if np.all(np.real(r) < 0.0):
        stable_K.append(K)

print("Approximate stable K range from scan:",
      min(stable_K), "to", max(stable_K))
