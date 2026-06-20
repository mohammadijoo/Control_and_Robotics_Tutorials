"""
Chapter30_Lesson2.py
Coordinate selection, diagonal scaling, and numerical conditioning
for state-space systems.

Dependencies:
    pip install numpy scipy control

The script compares a badly scaled physical-coordinate realization with a
diagonally scaled realization and shows how controllability/observability
matrices, Gramians, and pole placement become numerically better behaved.
"""

import numpy as np
from numpy.linalg import cond, eigvals
from scipy.linalg import solve_continuous_lyapunov as lyap
from scipy.signal import place_poles

try:
    import control
    HAS_CONTROL = True
except Exception:
    HAS_CONTROL = False


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Apow = np.eye(n)
    for _ in range(1, n):
        Apow = Apow @ A
        blocks.append(Apow @ B)
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return stacked [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = [C]
    Apow = np.eye(n)
    for _ in range(1, n):
        Apow = Apow @ A
        blocks.append(C @ Apow)
    return np.vstack(blocks)


def diagonal_state_scaling(A, B, C, x_nom):
    """
    Use z = S^{-1} x with S = diag(x_nom).
    Then A_z = S^{-1} A S, B_z = S^{-1} B, C_z = C S.
    """
    S = np.diag(x_nom)
    Sinv = np.diag(1.0 / x_nom)
    return Sinv @ A @ S, Sinv @ B, C @ S, S, Sinv


def gramian_condition_numbers(A, B, C):
    """
    For stable A:
        A Wc + Wc A^T + B B^T = 0
        A^T Wo + Wo A + C^T C = 0
    """
    Wc = lyap(A, -(B @ B.T))
    Wo = lyap(A.T, -(C.T @ C))
    return cond(Wc), cond(Wo), Wc, Wo


def main():
    # A simple electromechanical model with states of very different magnitudes:
    # x1 = position [m], x2 = velocity [m/s], x3 = current [A].
    A = np.array([
        [0.0,       1.0,       0.0],
        [-2.0e3,   -5.0e1,    8.0e4],
        [0.0,      -2.0e-2,  -4.0e3]
    ], dtype=float)
    B = np.array([[0.0], [0.0], [2.0e3]], dtype=float)
    C = np.array([[1.0, 0.0, 0.0]], dtype=float)

    # Nominal magnitudes chosen from engineering units or preliminary simulations.
    x_nom = np.array([1.0e-3, 1.0e-1, 1.0e1], dtype=float)
    Az, Bz, Cz, S, Sinv = diagonal_state_scaling(A, B, C, x_nom)

    print("Eigenvalues are invariant under similarity:")
    print("eig(A)  =", np.sort_complex(eigvals(A)))
    print("eig(Az) =", np.sort_complex(eigvals(Az)))

    Mc = controllability_matrix(A, B)
    Mcz = controllability_matrix(Az, Bz)
    Mo = observability_matrix(A, C)
    Moz = observability_matrix(Az, Cz)

    print("\nCondition numbers:")
    print(f"cond(A)                  = {cond(A):.3e}")
    print(f"cond(Az)                 = {cond(Az):.3e}")
    print(f"cond(controllability)    = {cond(Mc):.3e}")
    print(f"cond(scaled controll.)   = {cond(Mcz):.3e}")
    print(f"cond(observability)      = {cond(Mo):.3e}")
    print(f"cond(scaled observ.)     = {cond(Moz):.3e}")

    # Stable-system Gramian conditioning.
    kWc, kWo, Wc, Wo = gramian_condition_numbers(A, B, C)
    kWcz, kWoz, Wcz, Woz = gramian_condition_numbers(Az, Bz, Cz)
    print(f"cond(Wc), cond(Wo)       = {kWc:.3e}, {kWo:.3e}")
    print(f"cond(Wc_z), cond(Wo_z)   = {kWcz:.3e}, {kWoz:.3e}")

    # Pole placement in scaled coordinates. Convert Kz back to physical coordinates:
    # u = -Kz z = -Kz S^{-1} x, so Kx = Kz S^{-1}.
    desired_poles = np.array([-20.0, -35.0, -1200.0])
    Kz = place_poles(Az, Bz, desired_poles, method="YT").gain_matrix
    Kx_from_scaled = Kz @ Sinv
    closed_loop_poles = eigvals(A - B @ Kx_from_scaled)

    print("\nPole placement using scaled coordinates:")
    print("Kz =", Kz)
    print("Kx =", Kx_from_scaled)
    print("eig(A - B Kx) =", np.sort_complex(closed_loop_poles))

    if HAS_CONTROL:
        sys = control.ss(A, B, C, 0.0)
        sys_z = control.ss(Az, Bz, Cz, 0.0)
        print("\npython-control DC gains:")
        print(control.dcgain(sys), control.dcgain(sys_z))
        try:
            # balred/balanced realizations require optional slycot in many installs.
            hsv = control.hsvd(sys)
            print("Hankel singular values:", hsv)
        except Exception as exc:
            print("Hankel singular values unavailable:", exc)


if __name__ == "__main__":
    main()
