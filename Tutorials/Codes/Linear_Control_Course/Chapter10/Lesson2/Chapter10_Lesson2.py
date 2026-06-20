import numpy as np
import control  # python-control package
import matplotlib.pyplot as plt

# DC-motor-like plant G(s) = 1 / (s (s + 2))
num = [1.0]
den = [1.0, 2.0, 0.0]
G = control.TransferFunction(num, den)

# 1) Visualize the root locus
plt.figure()
control.root_locus(G)  # internally varies K >= 0
plt.title("Root locus of G(s) = 1 / (s (s + 2))")
plt.xlabel("Real axis")
plt.ylabel("Imag axis")
plt.grid(True)

# 2) Given the analytic relation, overshoot depends on K as:
#    Mp(K) = 100 * exp(-pi / sqrt(K - 1)),  K > 1.
def overshoot_from_K(K):
    if K <= 1.0:
        return 0.0  # no oscillation, purely overdamped
    return 100.0 * np.exp(-np.pi / np.sqrt(K - 1.0))

Mp_max = 10.0  # 10 percent
K_grid = np.linspace(1.01, 20.0, 2000)
Mp_vals = np.array([overshoot_from_K(K) for K in K_grid])

# 3) Find the smallest K satisfying Mp(K) <= Mp_max
K_candidates = K_grid[Mp_vals <= Mp_max]
K_star = K_candidates[0] if len(K_candidates) > 0 else None
print(f"Selected gain K* ≈ {K_star:.3f}")

# 4) Construct the closed-loop system and inspect poles
if K_star is not None:
    T = control.feedback(K_star * G, 1)  # unity feedback
    print("Closed-loop poles:", control.pole(T))

    # Step response
    t = np.linspace(0, 10, 1000)
    t, y = control.step_response(T, T=t)

    plt.figure()
    plt.plot(t, y)
    plt.title(f"Step response with K ≈ {K_star:.3f}")
    plt.xlabel("Time [s]")
    plt.ylabel("Output")
    plt.grid(True)

plt.show()
