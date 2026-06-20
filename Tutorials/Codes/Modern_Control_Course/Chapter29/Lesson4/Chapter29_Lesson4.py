# Chapter29_Lesson4.py
# Numerical controllability and observability Gramians for a 2-state LTV system.
#
# Requires: numpy
# Optional interpretation:
#   det(Wc) > 0  => reachable/controllable over the tested finite interval.
#   det(Wo) > 0  => observable over the tested finite interval.
#
# The example uses
#   x_dot = A(t)x + B(t)u
#   y     = C(t)x

import numpy as np


def A(t: float) -> np.ndarray:
    """Time-varying state matrix."""
    return np.array(
        [
            [0.0, 1.0],
            [-(2.0 + 0.4 * np.sin(t)), -(0.25 + 0.10 * np.cos(2.0 * t))],
        ],
        dtype=float,
    )


def B(t: float) -> np.ndarray:
    """Time-varying input matrix."""
    return np.array([[0.0], [1.0 + 0.25 * np.sin(0.5 * t)]], dtype=float)


def C(t: float) -> np.ndarray:
    """Time-varying output matrix."""
    return np.array([[1.0, 0.30 * np.cos(t)]], dtype=float)


def rk4_step_phi(phi: np.ndarray, t: float, h: float) -> np.ndarray:
    """One RK4 step for Phi_dot(t,t0)=A(t)Phi(t,t0)."""
    def f(tt: float, pp: np.ndarray) -> np.ndarray:
        return A(tt) @ pp

    k1 = f(t, phi)
    k2 = f(t + 0.5 * h, phi + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, phi + 0.5 * h * k2)
    k4 = f(t + h, phi + h * k3)
    return phi + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def compute_phi_grid(t0: float, tf: float, n_steps: int) -> tuple[np.ndarray, np.ndarray]:
    """Return grid times and Phi(t,t0) along the interval."""
    times = np.linspace(t0, tf, n_steps + 1)
    phi = np.eye(2)
    phis = [phi.copy()]

    for k in range(n_steps):
        h = times[k + 1] - times[k]
        phi = rk4_step_phi(phi, times[k], h)
        phis.append(phi.copy())

    return times, np.array(phis)


def trapezoid_gramians(t0: float, tf: float, n_steps: int = 4000) -> tuple[np.ndarray, np.ndarray]:
    """Compute finite-interval LTV controllability and observability Gramians."""
    times, phis = compute_phi_grid(t0, tf, n_steps)
    phi_tf_t0 = phis[-1]

    wc = np.zeros((2, 2))
    wo = np.zeros((2, 2))

    for k, s in enumerate(times):
        weight = 0.5 if k == 0 or k == len(times) - 1 else 1.0
        h = (tf - t0) / n_steps

        phi_s_t0 = phis[k]
        phi_tf_s = phi_tf_t0 @ np.linalg.inv(phi_s_t0)

        bs = B(float(s))
        cs = C(float(s))

        wc += weight * h * (phi_tf_s @ bs @ bs.T @ phi_tf_s.T)
        wo += weight * h * (phi_s_t0.T @ cs.T @ cs @ phi_s_t0)

    return wc, wo


def symmetric_eigs(M: np.ndarray) -> np.ndarray:
    """Eigenvalues for a symmetric Gramian, sorted increasingly."""
    return np.sort(np.linalg.eigvalsh(0.5 * (M + M.T)))


def main() -> None:
    t0, tf = 0.0, 6.0
    wc, wo = trapezoid_gramians(t0, tf)

    print("Finite interval:", (t0, tf))
    print("\nControllability Gramian Wc:")
    print(wc)
    print("eig(Wc):", symmetric_eigs(wc))
    print("det(Wc):", np.linalg.det(wc))
    print("rank(Wc):", np.linalg.matrix_rank(wc, tol=1e-8))

    print("\nObservability Gramian Wo:")
    print(wo)
    print("eig(Wo):", symmetric_eigs(wo))
    print("det(Wo):", np.linalg.det(wo))
    print("rank(Wo):", np.linalg.matrix_rank(wo, tol=1e-8))

    if np.linalg.matrix_rank(wc, tol=1e-8) == 2:
        print("\nThe tested interval is numerically controllable/reachable.")
    else:
        print("\nThe tested interval is numerically uncontrollable in at least one direction.")

    if np.linalg.matrix_rank(wo, tol=1e-8) == 2:
        print("The tested interval is numerically observable.")
    else:
        print("The tested interval is numerically unobservable in at least one direction.")


if __name__ == "__main__":
    main()
