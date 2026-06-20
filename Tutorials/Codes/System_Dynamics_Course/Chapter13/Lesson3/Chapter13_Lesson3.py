"""
Chapter13_Lesson3.py
System Dynamics (Control Engineering) — Chapter 13, Lesson 3
Modal Coordinates and Decoupling of MDOF Systems (Undamped Case)

This script:
1) Builds a 3-DOF mass-spring chain (M, K)
2) Solves the generalized eigenproblem K phi = lambda M phi
3) Mass-normalizes modes so that Phi^T M Phi = I and Phi^T K Phi = Omega^2
4) Simulates forced response using modal coordinates (decoupled ODEs)
5) Validates against direct physical-coordinate simulation

Dependencies:
  numpy, scipy, matplotlib
"""
import numpy as np
from numpy.linalg import norm
from scipy.linalg import eigh
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def mass_normalize(Phi: np.ndarray, M: np.ndarray) -> np.ndarray:
    """Scale columns of Phi so that Phi^T M Phi = I."""
    Phi_n = Phi.copy()
    for i in range(Phi.shape[1]):
        mi = Phi[:, i].T @ M @ Phi[:, i]
        Phi_n[:, i] = Phi[:, i] / np.sqrt(mi)
    return Phi_n


def build_3dof_chain(m=(1.0, 1.2, 0.9), k=(1200.0, 900.0, 700.0, 1100.0)):
    """
    3 masses connected by 4 springs:
      wall --k1-- m1 --k2-- m2 --k3-- m3 --k4-- wall
    """
    m1, m2, m3 = m
    k1, k2, k3, k4 = k
    M = np.diag([m1, m2, m3])
    K = np.array([
        [k1 + k2,   -k2,       0.0],
        [-k2,     k2 + k3,   -k3],
        [0.0,       -k3,     k3 + k4]
    ], dtype=float)
    return M, K


def modal_matrices(M: np.ndarray, K: np.ndarray):
    """
    Solve K phi = lambda M phi (symmetric generalized EVP).
    Returns (omega, Phi_mass_norm, Omega2_diag)
    """
    # eigh returns eigenvalues in ascending order for symmetric generalized EVP
    lam, Phi = eigh(K, M)
    lam = np.maximum(lam, 0.0)
    omega = np.sqrt(lam)
    Phi = mass_normalize(Phi, M)
    Omega2 = Phi.T @ K @ Phi
    return omega, Phi, Omega2


def simulate_modal(omega, Phi, force_fun, t_span=(0.0, 5.0), x0=None, v0=None, n_eval=2000):
    """
    Modal ODE: qdd + Omega^2 q = p(t), where p(t) = Phi^T f(t) (mass-normalized).
    """
    n = Phi.shape[0]
    if x0 is None:
        x0 = np.zeros(n)
    if v0 is None:
        v0 = np.zeros(n)

    # modal initial conditions: q0 = Phi^T M x0, qd0 = Phi^T M v0
    # since mass-normalized: Phi^T M Phi = I, and x = Phi q
    # thus q0 = Phi^T M x0, etc.
    # compute M from orthogonality identity: M = (Phi^{-T}) (Phi^{-1}) is not stable;
    # instead pass M in directly if needed. Here we reconstruct q0 by least squares:
    # x0 = Phi q0 -> q0 = argmin ||Phi q - x0||; same for v0.
    q0, *_ = np.linalg.lstsq(Phi, x0, rcond=None)
    qd0, *_ = np.linalg.lstsq(Phi, v0, rcond=None)

    def ode(t, z):
        q = z[:n]
        qd = z[n:]
        f = force_fun(t)  # physical force
        p = Phi.T @ f      # modal force
        qdd = - (omega**2) * q + p
        return np.hstack([qd, qdd])

    t_eval = np.linspace(t_span[0], t_span[1], n_eval)
    z0 = np.hstack([q0, qd0])
    sol = solve_ivp(ode, t_span, z0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    q = sol.y[:n, :]
    x = Phi @ q
    return sol.t, x, q


def simulate_physical(M, K, force_fun, t_span=(0.0, 5.0), x0=None, v0=None, n_eval=2000):
    """
    Physical ODE: M xdd + K x = f(t)  (undamped).
    Convert to first-order and integrate.
    """
    n = M.shape[0]
    if x0 is None:
        x0 = np.zeros(n)
    if v0 is None:
        v0 = np.zeros(n)
    Minv = np.linalg.inv(M)

    def ode(t, z):
        x = z[:n]
        xd = z[n:]
        f = force_fun(t)
        xdd = Minv @ (f - K @ x)
        return np.hstack([xd, xdd])

    t_eval = np.linspace(t_span[0], t_span[1], n_eval)
    z0 = np.hstack([x0, v0])
    sol = solve_ivp(ode, t_span, z0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    x = sol.y[:n, :]
    return sol.t, x


def main():
    M, K = build_3dof_chain()

    omega, Phi, Omega2 = modal_matrices(M, K)
    print("Natural frequencies (rad/s):", omega)
    print("Check Phi^T M Phi ~ I:\n", Phi.T @ M @ Phi)
    print("Check Phi^T K Phi ~ diag(omega^2):\n", Omega2)

    F0 = 10.0
    Omega = omega[0] * 0.9  # excite near first mode

    def f(t):
        return np.array([F0 * np.sin(Omega * t), 0.0, 0.0])

    t_span = (0.0, 8.0)
    x0 = np.zeros(3)
    v0 = np.zeros(3)

    t, x_modal, q = simulate_modal(omega, Phi, f, t_span=t_span, x0=x0, v0=v0)
    t2, x_phys = simulate_physical(M, K, f, t_span=t_span, x0=x0, v0=v0)

    # Plot
    plt.figure()
    plt.plot(t, x_modal[0, :], label="x1 (modal)")
    plt.plot(t2, x_phys[0, :], "--", label="x1 (physical)")
    plt.xlabel("t [s]")
    plt.ylabel("Displacement [m]")
    plt.legend()
    plt.title("Validation: modal vs physical simulation (DOF 1)")

    plt.figure()
    for i in range(3):
        plt.plot(t, q[i, :], label=f"q{i+1}")
    plt.xlabel("t [s]")
    plt.ylabel("Modal coordinate")
    plt.legend()
    plt.title("Modal coordinates q(t)")

    plt.show()


if __name__ == "__main__":
    main()
