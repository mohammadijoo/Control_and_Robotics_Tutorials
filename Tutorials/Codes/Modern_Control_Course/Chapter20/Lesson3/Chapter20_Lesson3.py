# Chapter20_Lesson3.py
# Internal vs external equivalence of continuous-time LTI systems.
# Required library: numpy

import numpy as np


def ctrb(A, B):
    """Kalman controllability matrix [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(blocks)


def obsv(A, C):
    """Kalman observability matrix [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def rank(M, tol=1e-10):
    return np.linalg.matrix_rank(M, tol=tol)


def transfer_value(A, B, C, D, s):
    """Evaluate G(s) = C(sI-A)^(-1)B + D."""
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def markov_parameters(A, B, C, D, count=6):
    """Return D and the first count continuous-time Markov matrices C A^k B."""
    n = A.shape[0]
    params = [D.copy()]
    Ak = np.eye(n)
    for _ in range(count):
        params.append(C @ Ak @ B)
        Ak = A @ Ak
    return params


def is_internally_equivalent(A1, B1, C1, D1, A2, B2, C2, D2, T, tol=1e-9):
    """Check A2=T A1 T^(-1), B2=T B1, C2=C1 T^(-1), D2=D1."""
    Ti = np.linalg.inv(T)
    tests = {
        "A2 = T A1 T^(-1)": np.linalg.norm(A2 - T @ A1 @ Ti),
        "B2 = T B1": np.linalg.norm(B2 - T @ B1),
        "C2 = C1 T^(-1)": np.linalg.norm(C2 - C1 @ Ti),
        "D2 = D1": np.linalg.norm(D2 - D1),
    }
    return tests, all(v < tol for v in tests.values())


def compare_transfers(systems, sample_points):
    """Compare scalar transfer functions at real sample points."""
    base = systems[0]
    rows = []
    for s in sample_points:
        g0 = transfer_value(*base, s=s)
        row = [float(g0[0, 0])]
        for sys in systems[1:]:
            gi = transfer_value(*sys, s=s)
            row.append(float(gi[0, 0]))
        rows.append((s, row))
    return rows


def print_system_report(name, A, B, C, D):
    n = A.shape[0]
    print(f"\n{name}")
    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("D =\n", D)
    print("rank controllability =", rank(ctrb(A, B)), "of", n)
    print("rank observability   =", rank(obsv(A, C)), "of", n)
    print("minimal?", rank(ctrb(A, B)) == n and rank(obsv(A, C)) == n)
    print("eigenvalues(A) =", np.linalg.eigvals(A))


if __name__ == "__main__":
    # A minimal two-state SISO realization.
    A1 = np.array([[-1.0, 0.0],
                   [ 0.0,-2.0]])
    B1 = np.array([[1.0],
                   [1.0]])
    C1 = np.array([[1.0, 1.0]])
    D1 = np.array([[0.0]])

    # Internal equivalence: choose a nonsingular state-coordinate map x2 = T x1.
    T = np.array([[1.0, 2.0],
                  [0.5, 1.5]])
    A2 = T @ A1 @ np.linalg.inv(T)
    B2 = T @ B1
    C2 = C1 @ np.linalg.inv(T)
    D2 = D1.copy()

    # External equivalence only: append a hidden mode. This mode is neither
    # reachable from u nor visible in y, so it does not change G(s).
    A3 = np.array([[-1.0, 0.0, 0.0],
                   [ 0.0,-2.0, 0.0],
                   [ 0.0, 0.0, 5.0]])
    B3 = np.array([[1.0],
                   [1.0],
                   [0.0]])
    C3 = np.array([[1.0, 1.0, 0.0]])
    D3 = D1.copy()

    # Another externally equivalent nonminimal realization with a different hidden mode.
    A4 = np.array([[-1.0, 0.0,  0.0],
                   [ 0.0,-2.0,  0.0],
                   [ 0.0, 0.0,-10.0]])
    B4 = B3.copy()
    C4 = C3.copy()
    D4 = D1.copy()

    for item in [("Sigma_1 minimal", A1, B1, C1, D1),
                 ("Sigma_2 internally equivalent to Sigma_1", A2, B2, C2, D2),
                 ("Sigma_3 externally equivalent but nonminimal", A3, B3, C3, D3),
                 ("Sigma_4 externally equivalent but different hidden eigenvalue", A4, B4, C4, D4)]:
        print_system_report(*item)

    tests, ok = is_internally_equivalent(A1, B1, C1, D1, A2, B2, C2, D2, T)
    print("\nInternal-equivalence residuals for Sigma_1 and Sigma_2:")
    for k, v in tests.items():
        print(f"  {k:22s}: {v:.3e}")
    print("Internally equivalent?", ok)

    print("\nTransfer-function samples G_i(s):")
    systems = [(A1, B1, C1, D1), (A2, B2, C2, D2), (A3, B3, C3, D3), (A4, B4, C4, D4)]
    for s, values in compare_transfers(systems, [0.1, 1.0, 3.0, 10.0]):
        print(f"s={s:4.1f} ->", values)

    print("\nFirst Markov parameters D, C B, C A B, ... for Sigma_1 and Sigma_3:")
    for k, (m1, m3) in enumerate(zip(markov_parameters(A1, B1, C1, D1),
                                     markov_parameters(A3, B3, C3, D3))):
        print(f"k={k}: Sigma_1 {m1.ravel()}   Sigma_3 {m3.ravel()}")
