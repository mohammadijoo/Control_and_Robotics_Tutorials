"""
Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
Lesson 1: Natural Frequencies and Normal Modes (Eigenvalue Problems)

This script:
  1) Builds (M, K) for a 3-DOF undamped mass-spring chain.
  2) Solves the symmetric generalized eigenproblem: K phi = (w^2) M phi.
  3) Mass-normalizes modes so that Phi^T M Phi = I and Phi^T K Phi = diag(w^2).
  4) Simulates free vibration via modal superposition for given initial conditions.
"""

import numpy as np

def mdof_chain_mk(masses, springs):
    """
    3-DOF chain (generalizable):
      wall --k1-- m1 --k2-- m2 --k3-- m3 -- (free end)
    K is assembled with end conditions matching the picture above.
    """
    n = len(masses)
    M = np.diag(masses).astype(float)

    # Springs: k1 between wall and m1; k2 between m1 and m2; ...; kn between m(n-1) and mn
    # Here springs length should be n (k1..kn)
    if len(springs) != n:
        raise ValueError("springs must have length n (k1..kn).")

    K = np.zeros((n, n), dtype=float)
    for i in range(n):
        # left spring contribution
        K[i, i] += springs[i]
        if i > 0:
            K[i, i] += springs[i-1]
            K[i, i-1] -= springs[i-1]
            K[i-1, i] -= springs[i-1]
    return M, K

def generalized_modes(M, K):
    """
    Solve K phi = (w^2) M phi for symmetric (M,K) with M SPD and K SPSD/SPD.

    Uses dense approach:
      - Convert to standard symmetric eigenproblem via Cholesky: M = L L^T
      - Solve (L^-1 K L^-T) u = (w^2) u
      - Recover phi = L^-T u
    """
    # Cholesky factor
    L = np.linalg.cholesky(M)

    # A = L^-1 K L^-T (symmetric)
    Linv = np.linalg.inv(L)
    A = Linv @ K @ Linv.T

    # Standard symmetric eigenproblem
    w2, U = np.linalg.eigh(A)  # ascending
    # Numerical cleanup for tiny negative values
    w2 = np.maximum(w2, 0.0)
    w = np.sqrt(w2)

    # Recover modes in physical coordinates
    Phi = np.linalg.solve(L.T, U)  # L^T Phi = U

    # Mass-normalize: phi_i^T M phi_i = 1
    for i in range(Phi.shape[1]):
        mi = Phi[:, i].T @ M @ Phi[:, i]
        Phi[:, i] = Phi[:, i] / np.sqrt(mi)

    return w, Phi

def check_orthogonality(M, K, w, Phi, tol=1e-8):
    Mt = Phi.T @ M @ Phi
    Kt = Phi.T @ K @ Phi
    # Expected: Mt ~ I, Kt ~ diag(w^2)
    I = np.eye(Mt.shape[0])
    D = np.diag(w**2)
    err_M = np.linalg.norm(Mt - I, ord=np.inf)
    err_K = np.linalg.norm(Kt - D, ord=np.inf)
    return err_M, err_K, Mt, Kt

def modal_free_response(M, K, w, Phi, q0, v0, t):
    """
    Undamped free vibration:
      q(t) = Phi * eta(t), with Phi^T M Phi = I
      eta_i(t) = eta0_i cos(w_i t) + (etaDot0_i / w_i) sin(w_i t)
    where eta0 = Phi^T M q0 and etaDot0 = Phi^T M v0.
    """
    q0 = np.asarray(q0, dtype=float).reshape(-1)
    v0 = np.asarray(v0, dtype=float).reshape(-1)
    eta0 = Phi.T @ M @ q0
    etaDot0 = Phi.T @ M @ v0

    eta = np.zeros((len(w), len(t)))
    for i, wi in enumerate(w):
        if wi < 1e-12:
            # Rigid mode (if any): eta_i(t) = eta0_i + etaDot0_i * t
            eta[i, :] = eta0[i] + etaDot0[i] * t
        else:
            eta[i, :] = eta0[i]*np.cos(wi*t) + (etaDot0[i]/wi)*np.sin(wi*t)

    q = Phi @ eta
    return q, eta

def main():
    # Example parameters
    masses  = [2.0, 1.5, 1.0]          # kg
    springs = [200.0, 300.0, 250.0]    # N/m

    M, K = mdof_chain_mk(masses, springs)
    w, Phi = generalized_modes(M, K)

    print("Natural frequencies (rad/s):")
    for i, wi in enumerate(w, start=1):
        print(f"  w{i} = {wi:.6f}")

    err_M, err_K, Mt, Kt = check_orthogonality(M, K, w, Phi)
    print("\nCheck mass-orthonormality (Phi^T M Phi):\n", Mt)
    print("\nCheck stiffness diagonalization (Phi^T K Phi):\n", Kt)
    print(f"\nErrors (inf-norm): err_M={err_M:.3e}, err_K={err_K:.3e}")

    # Free response example
    t = np.linspace(0.0, 5.0, 1501)
    q0 = [0.02, 0.0, -0.01]   # meters
    v0 = [0.0, 0.0, 0.0]      # m/s
    q, eta = modal_free_response(M, K, w, Phi, q0, v0, t)

    # Plot (optional)
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        for i in range(q.shape[0]):
            plt.plot(t, q[i, :], label=f"q{i+1}(t)")
        plt.xlabel("t (s)")
        plt.ylabel("displacement (m)")
        plt.title("3-DOF undamped free vibration (modal superposition)")
        plt.legend()
        plt.grid(True)
        plt.show()
    except Exception as e:
        print("Plotting skipped:", e)

if __name__ == "__main__":
    main()
