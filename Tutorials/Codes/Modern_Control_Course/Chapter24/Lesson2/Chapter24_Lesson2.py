# Chapter24_Lesson2.py
# Degrees of freedom and closed-loop eigenstructure for MIMO pole placement.
# Model: x_dot = A x + B u, u = -K x, so A_cl = A - B K.
#
# Dependencies:
#   pip install numpy

import numpy as np


def eigenstructure_column(lam: float, gamma: float):
    """
    For the example system

        A = [[0, 1, 0],
             [0, 0, 1],
             [-2, -3, -4]],     B = [[0, 0],
                                      [1, 0],
                                      [0, 1]]

    solve (A - lam I) v = B z, where z = K v.

    The second input gives one free shaping parameter gamma = z1.
    We set v1 = 1 and z1 = gamma, then solve the three scalar equations.
    """
    v1 = 1.0
    v2 = lam
    v3 = lam * lam + gamma
    z1 = gamma
    z2 = -2.0 - 3.0 * lam + (-4.0 - lam) * v3
    return np.array([v1, v2, v3], dtype=float), np.array([z1, z2], dtype=float)


def design_gain(lambdas, gammas):
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [-2.0, -3.0, -4.0]])

    B = np.array([[0.0, 0.0],
                  [1.0, 0.0],
                  [0.0, 1.0]])

    V_cols = []
    Z_cols = []
    for lam, gamma in zip(lambdas, gammas):
        v, z = eigenstructure_column(lam, gamma)
        V_cols.append(v)
        Z_cols.append(z)

    V = np.column_stack(V_cols)
    Z = np.column_stack(Z_cols)

    if abs(np.linalg.det(V)) < 1e-10:
        raise ValueError("Chosen eigenvectors are nearly singular. Change gammas.")

    K = Z @ np.linalg.inv(V)
    Acl = A - B @ K

    return A, B, V, Z, K, Acl


def main():
    desired_poles = np.array([-1.0, -2.0, -5.0])

    # These gamma values are the extra eigenvector-shaping degrees of freedom.
    # Keeping the same poles but changing gamma changes K and the eigenvectors.
    gammas = np.array([0.2, -0.4, 0.8])

    A, B, V, Z, K, Acl = design_gain(desired_poles, gammas)

    residual = A @ V - B @ Z - V @ np.diag(desired_poles)

    print("A =\n", A)
    print("B =\n", B)
    print("Desired poles =", desired_poles)
    print("Eigenvector-shaping gammas =", gammas)
    print("V =\n", V)
    print("Z = K V =\n", Z)
    print("K =\n", K)
    print("eig(A - B K) =", np.linalg.eigvals(Acl))
    print("condition number of V =", np.linalg.cond(V))
    print("max eigenstructure residual =", np.max(np.abs(residual)))


if __name__ == "__main__":
    main()
