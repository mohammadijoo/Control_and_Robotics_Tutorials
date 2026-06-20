"""
Chapter17_Lesson4.py
Transformations Between Physical and Modal Coordinates

This script demonstrates the coordinate transformation
    x = T z,     z = T^{-1} x
for a continuous-time LTI system
    x_dot = A x + B u,     y = C x + D u.

For diagonalizable A with T formed from eigenvectors, the modal system is
    z_dot = Lambda z + Bm u,    y = Cm z + D u,
where Lambda = T^{-1} A T, Bm = T^{-1} B, Cm = C T.
"""

import numpy as np


def modal_transform(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
    """Return T, T_inv, Lambda, Bm, Cm, D for a diagonalizable matrix A."""
    eigvals, T = np.linalg.eig(A)
    if abs(np.linalg.det(T)) < 1e-10:
        raise ValueError("Eigenvector matrix is nearly singular; A is not safely diagonalizable.")
    T_inv = np.linalg.inv(T)
    Lambda = T_inv @ A @ T
    Bm = T_inv @ B
    Cm = C @ T
    return eigvals, T, T_inv, Lambda, Bm, Cm, D


def transfer_value(A, B, C, D, s: complex):
    """Compute G(s) = C(sI-A)^(-1)B + D for a SISO example."""
    n = A.shape[0]
    return C @ np.linalg.inv(s * np.eye(n) - A) @ B + D


def main():
    # Physical coordinates: x = [position-like state; velocity-like state]
    A = np.array([[0.0, 1.0], [-2.0, -3.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    eigvals, T, T_inv, Lambda, Bm, Cm, Dm = modal_transform(A, B, C, D)

    print("Eigenvalues:")
    print(eigvals)
    print("\nT (columns are eigenvectors):")
    print(T)
    print("\nT_inv:")
    print(T_inv)
    print("\nLambda = T_inv A T:")
    print(Lambda)
    print("\nBm = T_inv B:")
    print(Bm)
    print("\nCm = C T:")
    print(Cm)

    # Transform an initial condition.
    x0 = np.array([[2.0], [-1.0]])
    z0 = T_inv @ x0
    x0_recovered = T @ z0
    print("\nx0:")
    print(x0)
    print("z0 = T_inv x0:")
    print(z0)
    print("T z0:")
    print(x0_recovered)

    # Verify transfer-function invariance at a complex frequency.
    s = 1.0 + 0.5j
    G_phys = transfer_value(A, B, C, D, s)
    G_modal = transfer_value(Lambda, Bm, Cm, Dm, s)
    print("\nG_phys(s):", G_phys)
    print("G_modal(s):", G_modal)
    print("difference norm:", np.linalg.norm(G_phys - G_modal))

    # Modal participation: y(t) = sum_i c_i exp(lambda_i t) z_i(0) for u=0.
    print("\nModal interpretation:")
    for i, lam in enumerate(eigvals):
        print(f"mode {i+1}: lambda={lam:.6g}, output weight C*T column={Cm[0, i]:.6g}, initial modal amplitude={z0[i, 0]:.6g}")


if __name__ == "__main__":
    main()
