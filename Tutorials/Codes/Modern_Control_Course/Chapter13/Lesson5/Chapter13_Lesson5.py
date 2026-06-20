# Chapter13_Lesson5.py
# Examples of observable and unobservable systems for continuous-time LTI models.
# Dependencies: numpy, scipy, matplotlib

import numpy as np
from numpy.linalg import matrix_rank
from scipy.linalg import expm
import matplotlib.pyplot as plt


def observability_matrix(A, C):
    """Build O = [C; C A; ...; C A^(n-1)]."""
    A = np.array(A, dtype=float)
    C = np.array(C, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def classify_system(name, A, C, stable_tol=1e-9):
    """Print rank, unobservable dimension, and a basic detectability indication."""
    A = np.array(A, dtype=float)
    C = np.array(C, dtype=float)
    O = observability_matrix(A, C)
    r = matrix_rank(O)
    n = A.shape[0]
    eigvals = np.linalg.eigvals(A)

    print("\n" + "=" * 72)
    print(name)
    print("A =\n", A)
    print("C =\n", C)
    print("Observability matrix O =\n", O)
    print(f"rank(O) = {r} out of n = {n}")
    print("eigenvalues(A) =", eigvals)

    # A simple numerical way to inspect unobservable directions:
    # right singular vectors associated with nearly zero singular values.
    U, s, Vh = np.linalg.svd(O)
    print("singular values(O) =", s)
    if r == n:
        print("Conclusion: observable.")
    else:
        null_vectors = Vh[r:, :].T
        print("Basis estimate for unobservable subspace columns =\n", null_vectors)
        # Detectability is exact only after isolating A-invariant unobservable modes.
        # For these examples, the null basis is one-dimensional and invariant.
        if null_vectors.shape[1] == 1:
            v = null_vectors[:, 0]
            Av = A @ v
            lam = (v @ Av) / (v @ v)
            residual = np.linalg.norm(Av - lam * v)
            print("candidate unobservable eigenvalue =", lam)
            print("invariance residual =", residual)
            if residual < 1e-7 and np.real(lam) < -stable_tol:
                print("Conclusion: unobservable but detectable.")
            elif residual < 1e-7:
                print("Conclusion: unobservable and not detectable.")
            else:
                print("Conclusion: unobservable; inspect invariant unobservable modes.")


def simulate_outputs(A, C, x0_list, t_grid):
    """Compare outputs from different initial states for u(t)=0."""
    A = np.array(A, dtype=float)
    C = np.array(C, dtype=float)
    outputs = []
    for x0 in x0_list:
        x0 = np.array(x0, dtype=float)
        y = []
        for t in t_grid:
            x = expm(A * t) @ x0
            y.append((C @ x).reshape(-1))
        outputs.append(np.array(y))
    return outputs


if __name__ == "__main__":
    # Example 1: observable second-order oscillator with position measurement.
    A1 = np.array([[0.0, 1.0],
                   [-2.0, -3.0]])
    C1 = np.array([[1.0, 0.0]])
    classify_system("Example 1: observable mass-spring-damper style system", A1, C1)

    # Example 2: unobservable decoupled state. x2 never affects x1 or y.
    A2 = np.array([[-1.0, 0.0],
                   [0.0, 2.0]])
    C2 = np.array([[1.0, 0.0]])
    classify_system("Example 2: unobservable and not detectable", A2, C2)

    # Example 3: unobservable but detectable. Hidden mode decays.
    A3 = np.array([[-1.0, 0.0],
                   [0.0, -4.0]])
    C3 = np.array([[1.0, 0.0]])
    classify_system("Example 3: unobservable but detectable", A3, C3)

    # Example 4: sensor placement changes observability.
    A4 = np.array([[0.0, 1.0],
                   [-6.0, -5.0]])
    C4_bad = np.array([[0.0, 1.0]])
    C4_good = np.array([[1.0, 0.0]])
    classify_system("Example 4a: velocity-only sensor for this model", A4, C4_bad)
    classify_system("Example 4b: position sensor for this model", A4, C4_good)

    # Output indistinguishability demonstration for Example 2:
    t_grid = np.linspace(0.0, 3.0, 200)
    x0_a = [1.0, 0.0]
    x0_b = [1.0, 5.0]
    y_a, y_b = simulate_outputs(A2, C2, [x0_a, x0_b], t_grid)

    plt.figure()
    plt.plot(t_grid, y_a[:, 0], label="x0 = [1, 0]")
    plt.plot(t_grid, y_b[:, 0], "--", label="x0 = [1, 5]")
    plt.xlabel("time")
    plt.ylabel("output y(t)")
    plt.title("Unobservable Example: Different Initial States, Same Output")
    plt.legend()
    plt.grid(True)
    plt.show()
