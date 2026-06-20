"""
Chapter21_Lesson5.py
Impact of transmission zeros on achievable performance and limitations.

This script compares a minimum-phase plant and a non-minimum-phase plant
with identical stable poles and identical DC gain. It demonstrates:
1. location of zeros,
2. inverse step response caused by a right-half-plane zero,
3. interpolation constraint T(z)=0 and S(z)=1 at an uncancelled RHP zero z.

Required packages:
    numpy scipy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


def eval_poly(coefficients, s):
    """Evaluate polynomial with coefficients in descending powers at complex s."""
    value = 0.0 + 0.0j
    for c in coefficients:
        value = value * s + c
    return value


def plant(num):
    den = [1.0, 5.0, 6.0]  # (s+2)(s+3)
    return signal.TransferFunction(num, den), den


def closed_loop_values(num, den, k, s):
    """
    Unity-feedback loop with L(s)=kP(s).
    T=L/(1+L), S=1/(1+L). At a plant zero z, L(z)=0, so T(z)=0 and S(z)=1.
    """
    p = eval_poly(num, s) / eval_poly(den, s)
    loop = k * p
    sensitivity = 1.0 / (1.0 + loop)
    complementary = loop / (1.0 + loop)
    return sensitivity, complementary


def main():
    # Same stable poles and same DC gain, different zero location.
    # G_min(s) = 6(s+1)/((s+2)(s+3)), zero at -1.
    # G_nmp(s) = 6(1-s)/((s+2)(s+3)), zero at +1.
    num_min = [6.0, 6.0]
    num_nmp = [-6.0, 6.0]
    sys_min, den = plant(num_min)
    sys_nmp, _ = plant(num_nmp)

    print("Minimum-phase zeros:", np.roots(num_min))
    print("Non-minimum-phase zeros:", np.roots(num_nmp))
    print("Common poles:", np.roots(den))

    t = np.linspace(0.0, 8.0, 1200)
    tout_min, y_min = signal.step(sys_min, T=t)
    tout_nmp, y_nmp = signal.step(sys_nmp, T=t)

    plt.figure(figsize=(9, 5))
    plt.plot(tout_min, y_min, label="zero at -1: minimum phase")
    plt.plot(tout_nmp, y_nmp, label="zero at +1: non-minimum phase")
    plt.axhline(1.0, linestyle="--", linewidth=1.0, label="unit steady value")
    plt.xlabel("time (s)")
    plt.ylabel("step response")
    plt.title("RHP zero produces inverse response and limits fast tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    z = 1.0  # RHP zero of G_nmp
    print("\nInterpolation at the non-minimum-phase zero z=1:")
    for k in [0.5, 2.0, 10.0, 100.0]:
        s_val, t_val = closed_loop_values(num_nmp, den, k, z)
        print(f"K={k:6.1f}  S(z)={s_val.real:.6f}  T(z)={t_val.real:.6f}")

    # A simple quantitative diagnostic: undershoot area for the NMP plant.
    undershoot = np.trapz(np.maximum(0.0, -y_nmp), t)
    print(f"\nApproximate negative undershoot area for NMP step response: {undershoot:.6f}")


if __name__ == "__main__":
    main()
