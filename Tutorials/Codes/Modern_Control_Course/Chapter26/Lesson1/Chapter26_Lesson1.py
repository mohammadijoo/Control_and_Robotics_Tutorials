"""
Chapter26_Lesson1.py

Need for steady-state accuracy in a state-space framework.
The script compares:
  1) state feedback with no reference prefilter,
  2) state feedback with static prefilter Nbar,
  3) the same prefilter under a constant matched input disturbance.

Dependencies:
  numpy, scipy, matplotlib

Optional control-engineering libraries to explore after this lesson:
  python-control: StateSpace, forced_response, dcgain, place
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def simulate(A, B, C, K, nbar, reference=1.0, disturbance=0.0, t_final=8.0):
    """Simulate xdot = (A-BK)x + B(nbar*r + d), y = Cx."""
    Acl = A - B @ K

    def rhs(t, x):
        u_total = float(nbar * reference + disturbance)
        return Acl @ x + (B[:, 0] * u_total)

    t_eval = np.linspace(0.0, t_final, 600)
    sol = solve_ivp(rhs, (0.0, t_final), np.zeros(A.shape[0]), t_eval=t_eval)
    y = (C @ sol.y).reshape(-1)
    return sol.t, y


def main():
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])
    K = np.array([[4.0, 2.0]])

    Acl = A - B @ K

    dc_closed = -C @ np.linalg.inv(Acl) @ B
    nbar = float(1.0 / dc_closed[0, 0])

    print("A-BK =")
    print(Acl)
    print("Closed-loop DC map from prefilter input v to y:", float(dc_closed[0, 0]))
    print("Required static prefilter Nbar:", nbar)

    cases = [
        ("Nbar = 1, no disturbance", 1.0, 1.0, 0.0),
        ("Nbar = computed, no disturbance", nbar, 1.0, 0.0),
        ("Nbar = computed, disturbance d = 0.2", nbar, 1.0, 0.2),
    ]

    for label, nb, r, d in cases:
        t, y = simulate(A, B, C, K, nb, reference=r, disturbance=d)
        print(f"{label}: final y approximately {y[-1]:.6f}, final error {r - y[-1]:.6f}")

    plt.figure()
    for label, nb, r, d in cases:
        t, y = simulate(A, B, C, K, nb, reference=r, disturbance=d)
        plt.plot(t, y, label=label)
    plt.axhline(1.0, linestyle="--", label="reference")
    plt.xlabel("time (s)")
    plt.ylabel("output y(t)")
    plt.title("Chapter 26 Lesson 1: steady-state accuracy")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
