# Chapter17_Lesson5.py
# Practical Considerations in Choosing Canonical Forms
# Requires: numpy, scipy (optional for matrix exponential demo)

import numpy as np

np.set_printoptions(precision=6, suppress=True)


def inv(M: np.ndarray) -> np.ndarray:
    """Numerically invert a square matrix."""
    return np.linalg.inv(M)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Build O = [C; C A; ...; C A^(n-1)]."""
    n = A.shape[0]
    rows = []
    Ak = np.eye(n)
    for _ in range(n):
        rows.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(rows)


def frobenius_condition(M: np.ndarray) -> float:
    """A simple conditioning proxy: ||M||_F ||M^{-1}||_F."""
    return np.linalg.norm(M, ord="fro") * np.linalg.norm(inv(M), ord="fro")


def transform_system(A: np.ndarray, B: np.ndarray, C: np.ndarray, T: np.ndarray):
    """For x = T z, return z-dot = Abar z + Bbar u, y = Cbar z."""
    Ti = inv(T)
    Abar = Ti @ A @ T
    Bbar = Ti @ B
    Cbar = C @ T
    return Abar, Bbar, Cbar


def choose_form(kappa_ocf: float, kappa_modal: float, observable_rank: int, n: int) -> str:
    """Rule-based recommendation for instructional use."""
    if observable_rank < n:
        return "Do not use OCF before separating the unobservable subspace."
    if kappa_modal < 50 and kappa_modal <= kappa_ocf:
        return "Modal form is preferred for mode interpretation and decoupled simulation."
    if kappa_ocf < 50 and kappa_ocf < kappa_modal:
        return "OCF is acceptable for observer-side algebra and transfer-function realization."
    return "Use the physical/scaled coordinates or a numerically orthogonal form instead."


def main() -> None:
    # Companion model with p(s) = s^3 + 6 s^2 + 11 s + 6 = (s+1)(s+2)(s+3)
    A0 = np.array([[0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [-6.0, -11.0, -6.0]])
    B0 = np.array([[0.0], [0.0], [1.0]])
    C0 = np.array([[1.0, 0.0, 0.0]])

    # A deliberately scaled coordinate system: x = S z0.
    S = np.diag([1.0, 0.05, 20.0])
    A = S @ A0 @ inv(S)
    B = S @ B0
    C = C0 @ inv(S)

    print("Physical/scaled A:")
    print(A)
    print("B =", B.T)
    print("C =", C)

    # Observable canonical target: A_o = A0^T, C_o = [0 0 1].
    Ao = A0.T
    Co = np.array([[0.0, 0.0, 1.0]])
    O = observability_matrix(A, C)
    Oo = observability_matrix(Ao, Co)
    T_ocf = inv(O) @ Oo
    A_ocf, B_ocf, C_ocf = transform_system(A, B, C, T_ocf)

    print("\nRank of observability matrix:", np.linalg.matrix_rank(O))
    print("Frobenius condition proxy of OCF transform:", frobenius_condition(T_ocf))
    print("A in observable canonical coordinates:")
    print(A_ocf)
    print("C in observable canonical coordinates:", C_ocf)

    # Modal transform: right eigenvectors of A.
    eigvals, V = np.linalg.eig(A)
    A_modal, B_modal, C_modal = transform_system(A, B, C, V)

    print("\nEigenvalues:", eigvals)
    print("Frobenius condition proxy of modal eigenvector matrix:", frobenius_condition(V))
    print("A in modal coordinates:")
    print(A_modal)

    print("\nRecommendation:")
    print(choose_form(frobenius_condition(T_ocf), frobenius_condition(V), np.linalg.matrix_rank(O), A.shape[0]))


if __name__ == "__main__":
    main()
