# Chapter9_Lesson3.py
# Modes, modal decomposition, and dominant modes for continuous-time LTI systems.
#
# Required libraries:
#   pip install numpy scipy matplotlib
#
# Optional control-engineering library:
#   pip install control
#
# This file studies x_dot = A x, y = C x by comparing:
#   1) full matrix-exponential response,
#   2) exact modal reconstruction using eigenvectors,
#   3) dominant-mode approximation.

import numpy as np
from scipy.linalg import expm, eig
import matplotlib.pyplot as plt


def modal_decomposition(A: np.ndarray):
    """Return eigenvalues, right eigenvectors, and left modal rows.

    If A is diagonalizable, A = V Lambda V^{-1}.
    The rows of W = V^{-1} are left modal coordinate functionals.
    Modal coordinate z_i(0) = w_i x(0).
    """
    lambdas, V = eig(A)
    W = np.linalg.inv(V)
    return lambdas, V, W


def full_response(A: np.ndarray, x0: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Compute x(t)=exp(A t)x0 for each t."""
    return np.array([expm(A * t) @ x0 for t in t_grid])


def modal_response(lambdas: np.ndarray, V: np.ndarray, W: np.ndarray,
                   x0: np.ndarray, t_grid: np.ndarray,
                   selected_modes=None) -> np.ndarray:
    """Reconstruct the state from selected modes.

    selected_modes=None uses all modes. For real systems with complex-conjugate
    pairs, the final real part is the physical state.
    """
    n = len(lambdas)
    if selected_modes is None:
        selected_modes = list(range(n))

    z0 = W @ x0
    xs = []
    for t in t_grid:
        x = np.zeros(V.shape[0], dtype=complex)
        for i in selected_modes:
            x += V[:, i] * np.exp(lambdas[i] * t) * z0[i]
        xs.append(np.real_if_close(x).real)
    return np.array(xs)


def dominant_modes_by_real_part(lambdas: np.ndarray, keep: int):
    """Return indices of modes with largest real parts."""
    return [int(i) for i in np.argsort(np.real(lambdas))[::-1][:keep]]


def modal_table(lambdas: np.ndarray, V: np.ndarray, W: np.ndarray,
                C: np.ndarray, x0: np.ndarray):
    """Build a readable modal table containing excitation and output residue."""
    z0 = W @ x0
    rows = []
    for i, lam in enumerate(lambdas):
        output_residue = C @ V[:, i] * z0[i]
        rows.append({
            "mode": i,
            "lambda": lam,
            "real_part": np.real(lam),
            "imag_part": np.imag(lam),
            "initial_modal_amplitude": z0[i],
            "output_residue": output_residue[0],
        })
    return rows


def main():
    # A stable third-order system:
    #   lambda_1,2 = -0.25 +/- 1.50 j  (slow oscillatory dominant pair)
    #   lambda_3   = -3.00             (fast real mode)
    A = np.array([
        [-0.25,  1.50, 0.0],
        [-1.50, -0.25, 0.0],
        [ 0.00,  0.00, -3.0],
    ], dtype=float)

    C = np.array([[1.0, 0.0, 0.4]])
    x0 = np.array([1.0, -0.2, 2.0])
    t_grid = np.linspace(0.0, 20.0, 800)

    lambdas, V, W = modal_decomposition(A)

    print("Eigenvalues:")
    for lam in lambdas:
        print(f"  {lam.real:+.4f} {lam.imag:+.4f}j")

    print("\nModal participation/output residue table:")
    for row in modal_table(lambdas, V, W, C, x0):
        print(
            f"mode={row['mode']}, lambda={row['lambda']:.4g}, "
            f"z0={row['initial_modal_amplitude']:.4g}, "
            f"output residue={row['output_residue']:.4g}"
        )

    x_full = full_response(A, x0, t_grid)
    x_modal = modal_response(lambdas, V, W, x0, t_grid)

    dominant = dominant_modes_by_real_part(lambdas, keep=2)
    x_dom = modal_response(lambdas, V, W, x0, t_grid, selected_modes=dominant)

    y_full = (C @ x_full.T).ravel()
    y_modal = (C @ x_modal.T).ravel()
    y_dom = (C @ x_dom.T).ravel()

    print("\nMax reconstruction error between expm and all-mode reconstruction:")
    print(np.max(np.abs(y_full - y_modal)))

    print("\nDominant mode indices:", dominant)
    print("Dominant eigenvalues:", lambdas[dominant])

    plt.figure(figsize=(9, 5))
    plt.plot(t_grid, y_full, label="full output y(t)")
    plt.plot(t_grid, y_dom, "--", label="dominant-mode approximation")
    plt.xlabel("time [s]")
    plt.ylabel("output")
    plt.title("Modal decomposition and dominant-mode approximation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(9, 5))
    plt.semilogy(t_grid, np.abs(y_full - y_dom) + 1e-14)
    plt.xlabel("time [s]")
    plt.ylabel("|full - dominant|")
    plt.title("Error caused by neglecting fast modes")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
