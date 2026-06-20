# Chapter10_Lesson5.py
"""
Modern Control - Chapter 10, Lesson 5
Examples of controllable and uncontrollable systems.

Libraries:
    numpy: matrix construction and rank computation.
    scipy.linalg: optional matrix utilities.
    python-control: optional; if installed, control.ctrb(A, B) gives the
    same controllability matrix used here.

Run:
    python Chapter10_Lesson5.py
"""

import numpy as np
from numpy.linalg import matrix_rank


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, A^2 B, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def analyze_system(name: str, A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> None:
    C = controllability_matrix(A, B)
    r = matrix_rank(C, tol=tol)
    n = A.shape[0]
    print(f"\n{name}")
    print("-" * len(name))
    print("A =\n", A)
    print("B =\n", B)
    print("Controllability matrix C =\n", C)
    print(f"rank(C) = {r} out of n = {n}")
    print("Conclusion:", "controllable" if r == n else "uncontrollable")


def main() -> None:
    # Example 1: double integrator, input is acceleration.
    A1 = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
    B1 = np.array([[0.0],
                   [1.0]])
    analyze_system("Example 1: double integrator", A1, B1)

    # Example 2: two decoupled integrator chains, only the first chain is actuated.
    A2 = np.array([[0.0, 1.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0],
                   [0.0, 0.0, 0.0, 0.0]])
    B2 = np.array([[0.0],
                   [1.0],
                   [0.0],
                   [0.0]])
    analyze_system("Example 2: two decoupled masses, one actuator", A2, B2)

    # Example 3: coupled two-mass system. Input acts on mass 1.
    # State x = [q1, v1, q2, v2]^T.
    m1, m2 = 1.0, 1.0
    k1, k2, kc = 1.0, 1.2, 0.8
    c1, c2 = 0.1, 0.2
    A3 = np.array([[0.0, 1.0, 0.0, 0.0],
                   [-(k1 + kc) / m1, -c1 / m1, kc / m1, 0.0],
                   [0.0, 0.0, 0.0, 1.0],
                   [kc / m2, 0.0, -(k2 + kc) / m2, -c2 / m2]])
    B3 = np.array([[0.0],
                   [1.0 / m1],
                   [0.0],
                   [0.0]])
    analyze_system("Example 3: coupled two-mass oscillator", A3, B3)

    # Example 4: diagonal modal system.
    # A mode is unreachable if its corresponding entry in B is zero.
    A4 = np.diag([-1.0, -2.0, -3.0])
    B4a = np.array([[1.0],
                    [1.0],
                    [1.0]])
    B4b = np.array([[1.0],
                    [0.0],
                    [1.0]])
    analyze_system("Example 4a: diagonal system, all modes actuated", A4, B4a)
    analyze_system("Example 4b: diagonal system, middle mode not actuated", A4, B4b)

    # Example 5: stable uncontrollable hidden mode.
    A5 = np.array([[-2.0, 0.0],
                   [0.0, -0.5]])
    B5 = np.array([[1.0],
                   [0.0]])
    analyze_system("Example 5: stable but uncontrollable hidden mode", A5, B5)

    # Optional: compare with python-control if installed.
    try:
        import control
        C_control = control.ctrb(A3, B3)
        print("\npython-control check for Example 3:")
        print("rank(control.ctrb(A3, B3)) =", matrix_rank(C_control))
    except ImportError:
        print("\nOptional package 'control' is not installed; custom implementation was used.")


if __name__ == "__main__":
    main()
