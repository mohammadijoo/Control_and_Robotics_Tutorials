# Chapter26_Lesson2.py
"""
Modern Control - Chapter 26, Lesson 2
Augmenting the State with Integral of Tracking Error

Requires: numpy, scipy
Optional: matplotlib for plotting the step response
"""

import numpy as np
from scipy.signal import place_poles, StateSpace, lsim
import matplotlib.pyplot as plt


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    return np.hstack(blocks)


def augment_with_error_integral(A, B, C, D=None):
    """
    Build the augmented servo model for q_dot = r - y.

    Plant:
        x_dot = A x + B u
        y     = C x + D u

    Integral state:
        q_dot = r - y = -C x - D u + r

    Augmented dynamics:
        xa_dot = Aa xa + Ba u + Br r,  xa = [x; q]
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    if D is None:
        D = np.zeros((C.shape[0], B.shape[1]))
    D = np.asarray(D, dtype=float)

    n = A.shape[0]
    p = C.shape[0]
    Aa = np.block([
        [A,                 np.zeros((n, p))],
        [-C,                np.zeros((p, p))]
    ])
    Ba = np.vstack([B, -D])
    Br = np.vstack([np.zeros((n, p)), np.eye(p)])
    Ca = np.hstack([C, np.zeros((p, p))])
    Da = D.copy()
    return Aa, Ba, Br, Ca, Da


def rank_tests(A, B, C, D=None):
    """Check plant controllability and the zero-at-origin servo condition."""
    if D is None:
        D = np.zeros((C.shape[0], B.shape[1]))
    plant_ctrb_rank = np.linalg.matrix_rank(controllability_matrix(A, B))
    rosenbrock_0 = np.block([[A, B], [C, D]])
    rosen_rank = np.linalg.matrix_rank(rosenbrock_0)
    return plant_ctrb_rank, rosen_rank, rosenbrock_0


def main():
    # Example: stable second-order plant, output is position.
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    Aa, Ba, Br, Ca, Da = augment_with_error_integral(A, B, C, D)
    print("Aa =\n", Aa)
    print("Ba =\n", Ba)
    print("Br =\n", Br)

    plant_rank, rosen_rank, R0 = rank_tests(A, B, C, D)
    print("rank controllability(A,B) =", plant_rank)
    print("rank Rosenbrock at zero =", rosen_rank, "required =", A.shape[0] + C.shape[0])
    print("Rosenbrock matrix at zero =\n", R0)

    aug_ctrb = controllability_matrix(Aa, Ba)
    print("rank controllability(Aa,Ba) =", np.linalg.matrix_rank(aug_ctrb))

    # Pole placement is formally Lesson 3, but it verifies that the augmented
    # model created in this lesson is design-ready.
    desired_poles = np.array([-2.0, -3.0, -4.0])
    Kaug = place_poles(Aa, Ba, desired_poles).gain_matrix
    Kx = Kaug[:, :A.shape[0]]
    Ki = -Kaug[:, A.shape[0]:]  # because q_dot = r - y and u = -Kx x + Ki q

    print("Kaug for u = -Kaug [x; q] =", Kaug)
    print("Equivalent Kx =", Kx)
    print("Equivalent Ki in u = -Kx x + Ki q =", Ki)

    Acl = Aa - Ba @ Kaug
    sys_cl = StateSpace(Acl, Br, Ca, np.zeros((1, 1)))

    t = np.linspace(0.0, 8.0, 500)
    r = np.ones_like(t)
    tout, yout, xout = lsim(sys_cl, U=r, T=t)
    steady_state_error = 1.0 - yout[-1]
    print("final output =", yout[-1])
    print("final tracking error =", steady_state_error)

    plt.figure()
    plt.plot(tout, yout, label="y(t)")
    plt.plot(tout, r, "--", label="r(t)")
    plt.xlabel("time [s]")
    plt.ylabel("output")
    plt.title("Integral-augmented state feedback: unit-step tracking")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
