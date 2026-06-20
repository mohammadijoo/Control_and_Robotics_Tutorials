# Chapter23_Lesson5.py
# Numerical Sensitivity and Conditioning in SISO Pole Placement
#
# Dependencies:
#   pip install numpy scipy matplotlib
#
# This script compares Ackermann pole placement under different state scalings
# and estimates closed-loop eigenvalue sensitivity under small model perturbations.

import numpy as np
from numpy.linalg import inv, cond, norm, eig
import scipy.linalg as la
import matplotlib.pyplot as plt


def controllability_matrix(A, B):
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    cols = []
    Ak = np.eye(n)
    for _ in range(n):
        cols.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(cols)


def polynomial_matrix(A, coeffs):
    """
    Evaluate p(A) for monic polynomial
        p(s) = s^n + coeffs[1] s^(n-1) + ... + coeffs[n]
    where np.poly returns [1, a_{n-1}, ..., a_0].
    """
    n = A.shape[0]
    P = np.zeros_like(A, dtype=float)
    for c in coeffs:
        P = P @ A + c * np.eye(n)
    return P


def ackermann_gain(A, B, desired_poles):
    """SISO Ackermann formula K = e_n^T C^{-1} p_d(A)."""
    n = A.shape[0]
    C = controllability_matrix(A, B)
    coeffs = np.poly(desired_poles)  # [1, alpha_{n-1}, ..., alpha_0]
    pA = polynomial_matrix(A, coeffs)
    eT = np.zeros((1, n))
    eT[0, -1] = 1.0
    K = eT @ inv(C) @ pA
    return K, C, coeffs


def eigenvalue_condition_numbers(M):
    """
    Compute eigenvalue condition numbers kappa_i = ||w_i|| ||v_i|| / |w_i^H v_i|.
    SciPy returns left eigenvectors satisfying w_i^H M = lambda_i w_i^H.
    """
    eigvals, VL, VR = la.eig(M, left=True, right=True)
    kappas = []
    for i in range(len(eigvals)):
        w = VL[:, i]
        v = VR[:, i]
        denom = abs(np.vdot(w, v))
        kappas.append((norm(w) * norm(v)) / denom)
    return eigvals, np.array(kappas)


def perturbation_experiment(A, B, K, eps=1e-7, trials=300, seed=1):
    """Randomly perturb A and B and measure closed-loop pole drift."""
    rng = np.random.default_rng(seed)
    M0 = A - B @ K
    lam0 = np.sort_complex(eig(M0)[0])
    drifts = []
    for _ in range(trials):
        dA = rng.normal(size=A.shape)
        dB = rng.normal(size=B.shape)
        dA *= eps * max(1.0, norm(A, 2)) / norm(dA, 2)
        dB *= eps * max(1.0, norm(B, 2)) / norm(dB, 2)
        M = (A + dA) - (B + dB) @ K
        lam = np.sort_complex(eig(M)[0])
        drifts.append(norm(lam - lam0, 2))
    return np.array(drifts)


def similarity_transform(A, B, T):
    """x = T z gives z_dot = T^{-1} A T z + T^{-1} B u."""
    Ti = inv(T)
    return Ti @ A @ T, Ti @ B


def main():
    np.set_printoptions(precision=6, suppress=True)

    # A controllable third-order example with states on unequal numerical scales.
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [-2.0, -3.0, -4.0]])
    B = np.array([[0.0], [0.0], [1.0]])

    desired_poles = np.array([-4.0, -5.0, -6.0])

    K, C, coeffs = ackermann_gain(A, B, desired_poles)
    M = A - B @ K
    eigvals, eig_kappa = eigenvalue_condition_numbers(M)
    drifts = perturbation_experiment(A, B, K)

    print("Original coordinates")
    print("--------------------")
    print("Desired polynomial coefficients:", coeffs)
    print("Controllability condition number:", cond(C))
    print("K =", K)
    print("Closed-loop eigenvalues:", eig(M)[0])
    print("Eigenvalue condition numbers:", eig_kappa)
    print("Median perturbation pole drift:", np.median(drifts))
    print("Max perturbation pole drift:", np.max(drifts))

    # Deliberately poor state scaling: x = T z.
    T_bad = np.diag([1e-3, 1.0, 1e3])
    Abad, Bbad = similarity_transform(A, B, T_bad)
    Kbad_z, Cbad, _ = ackermann_gain(Abad, Bbad, desired_poles)
    # Convert u = -Kz z = -Kx x, with z = T^{-1}x, so Kx = Kz T^{-1}.
    Kbad_x = Kbad_z @ inv(T_bad)
    Mbad = A - B @ Kbad_x
    eigvals_bad, eig_kappa_bad = eigenvalue_condition_numbers(Mbad)
    drifts_bad = perturbation_experiment(A, B, Kbad_x)

    print("\nBadly scaled design coordinates")
    print("-------------------------------")
    print("cond(C_bad) =", cond(Cbad))
    print("K_z in bad coordinates =", Kbad_z)
    print("Equivalent K_x =", Kbad_x)
    print("Closed-loop eigenvalues:", eig(Mbad)[0])
    print("Eigenvalue condition numbers:", eig_kappa_bad)
    print("Median perturbation pole drift:", np.median(drifts_bad))
    print("Max perturbation pole drift:", np.max(drifts_bad))

    labels = ["original", "bad scaling"]
    values = [np.log10(cond(C)), np.log10(cond(Cbad))]
    plt.figure()
    plt.bar(labels, values)
    plt.ylabel("log10 cond(controllability matrix)")
    plt.title("Coordinate scaling can destroy numerical conditioning")
    plt.tight_layout()
    plt.show()

    plt.figure()
    plt.boxplot([drifts, drifts_bad], labels=labels)
    plt.ylabel("2-norm closed-loop pole drift")
    plt.title("Pole-placement sensitivity under small perturbations")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
