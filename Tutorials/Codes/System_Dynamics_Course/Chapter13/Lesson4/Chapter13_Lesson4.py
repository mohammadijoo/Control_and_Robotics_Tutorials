"""
Chapter13_Lesson4.py
Damping in MDOF Systems and Mode Shapes with Damping

This script demonstrates:
1) Undamped modal analysis: K phi = (w^2) M phi, with mass-normalization.
2) Rayleigh (proportional) damping: C = alpha*M + beta*K, and modal damping ratios.
3) Non-proportional damping: state-space eigenanalysis -> complex poles and complex mode shapes.
4) Free response simulation for the non-proportional case.

Dependencies:
  pip install numpy scipy matplotlib
"""

import numpy as np

try:
    from scipy.linalg import eigh, eig
    from scipy.integrate import solve_ivp
except ImportError as e:
    raise SystemExit("SciPy is required. Install with: pip install scipy") from e


def chain_matrices_3dof(m1=1.0, m2=1.0, m3=1.0, k1=2000.0, k2=1500.0, k3=1000.0):
    """3-DOF shear/chain model with base fixed.
    DOF i is the displacement of mass i.
    Springs: k1 (ground-m1), k2 (m1-m2), k3 (m2-m3).
    """
    M = np.diag([m1, m2, m3])

    K = np.array([
        [k1 + k2, -k2,      0.0],
        [-k2,     k2 + k3, -k3 ],
        [0.0,     -k3,      k3 ]
    ], dtype=float)
    return M, K


def mass_normalize_modes(M, Phi):
    """Scale columns of Phi so that Phi^T M Phi = I."""
    PhiN = Phi.copy()
    for i in range(Phi.shape[1]):
        mi = Phi[:, i].T @ M @ Phi[:, i]
        PhiN[:, i] /= np.sqrt(mi)
    return PhiN


def rayleigh_from_two_targets(omega_i, zeta_i, omega_j, zeta_j):
    """Solve for Rayleigh coefficients alpha, beta using:
         zeta_r = alpha/(2*omega_r) + beta*omega_r/2
    at two frequencies (omega_i, omega_j).
    """
    # alpha + beta*omega^2 = 2*zeta*omega
    A = np.array([[1.0, omega_i**2],
                  [1.0, omega_j**2]], dtype=float)
    b = np.array([2.0*zeta_i*omega_i, 2.0*zeta_j*omega_j], dtype=float)
    alpha, beta = np.linalg.solve(A, b)
    return alpha, beta


def build_state_matrix(M, C, K):
    """Build first-order state matrix A for:
       M xdd + C xd + K x = 0
       z = [x; xd], z_dot = A z
    """
    n = M.shape[0]
    Minv = np.linalg.inv(M)
    Z = np.zeros((n, n))
    I = np.eye(n)
    A = np.block([
        [Z, I],
        [-Minv @ K, -Minv @ C]
    ])
    return A


def modal_damping_matrix(Phi, C):
    return Phi.T @ C @ Phi


def summarize_poles(lam, max_modes=6):
    """Return a list of (sigma, wd, wn, zeta) for conjugate pairs with wd>0."""
    out = []
    # keep only positive imaginary part poles (each mode once)
    lam_pos = [x for x in lam if np.imag(x) > 1e-8]
    lam_pos.sort(key=lambda x: np.imag(x))
    for x in lam_pos[:max_modes]:
        sigma = np.real(x)
        wd = np.imag(x)
        wn = np.sqrt(sigma**2 + wd**2)
        zeta = -sigma / wn if wn > 0 else np.nan
        out.append((sigma, wd, wn, zeta))
    return out


def main():
    np.set_printoptions(precision=5, suppress=True)

    # --------------------------
    # Example system (3-DOF)
    # --------------------------
    M, K = chain_matrices_3dof(m1=1.2, m2=1.0, m3=0.8, k1=2500.0, k2=1800.0, k3=1200.0)
    n = M.shape[0]

    # --------------------------
    # (A) Undamped modal analysis
    # --------------------------
    # For symmetric M, K with M positive definite:
    # scipy.linalg.eigh solves K v = lam M v
    lam, Phi = eigh(K, M)  # lam = omega^2
    omega = np.sqrt(lam)
    Phi = mass_normalize_modes(M, Phi)

    print("Undamped natural frequencies (rad/s):", omega)
    print("Check Phi^T M Phi (should be I):\n", Phi.T @ M @ Phi)
    print("Check Phi^T K Phi (should be diag(omega^2)):\n", Phi.T @ K @ Phi)

    # ------------------------------------
    # (B) Proportional damping (Rayleigh)
    # ------------------------------------
    # Target damping ratios at two modes (typical structural values)
    zeta_1 = 0.02  # 2% at mode 1
    zeta_3 = 0.05  # 5% at mode 3
    alpha, beta = rayleigh_from_two_targets(omega[0], zeta_1, omega[2], zeta_3)

    C_ray = alpha*M + beta*K
    Cm_ray = modal_damping_matrix(Phi, C_ray)

    print("\nRayleigh damping coefficients:")
    print("alpha =", alpha, " beta =", beta)
    print("Modal damping matrix Phi^T C Phi (Rayleigh) ~ diagonal:\n", Cm_ray)

    # Modal damping ratios predicted by Rayleigh:
    zeta_modal = 0.5*(alpha/omega + beta*omega)
    print("Modal damping ratios (Rayleigh):", zeta_modal)

    # ------------------------------------------
    # (C) Non-proportional damping (example)
    # ------------------------------------------
    # Build a localized damping that is not representable as alpha*M + beta*K:
    # Dashpot between DOF1 and DOF3 (non-adjacent coupling) + dashpot to ground at DOF2.
    c13 = 45.0
    c2g = 35.0
    C_np = np.zeros((n, n), dtype=float)
    # between 1 and 3
    C_np[0, 0] += c13
    C_np[2, 2] += c13
    C_np[0, 2] -= c13
    C_np[2, 0] -= c13
    # DOF2 to ground
    C_np[1, 1] += c2g

    # Compare modal damping matrix (should have off-diagonals)
    Cm_np = modal_damping_matrix(Phi, C_np)
    print("\nNon-proportional C (example):\n", C_np)
    print("Modal damping matrix Phi^T C Phi (non-proportional) has off-diagonals:\n", Cm_np)

    # State-space eigenanalysis => complex poles and complex mode shapes
    A = build_state_matrix(M, C_np, K)
    lamA, vA = eig(A)

    print("\nComplex poles (sigma + j*wd) for first few modes (wd>0):")
    poles = summarize_poles(lamA, max_modes=3)
    for i, (sigma, wd, wn, zeta) in enumerate(poles, start=1):
        print(f"Mode {i}: sigma={sigma:+.6f}, wd={wd:.6f}, wn={wn:.6f}, zeta={zeta:.6f}")

    # Extract one complex mode shape (displacement part) corresponding to smallest wd>0
    # Find index of pole with smallest positive imaginary part
    idx_candidates = np.where(np.imag(lamA) > 1e-8)[0]
    idx = idx_candidates[np.argmin(np.imag(lamA[idx_candidates]))]
    lam_sel = lamA[idx]
    v_sel = vA[:, idx]
    phi_c = v_sel[:n]  # displacement component
    # normalize by maximum magnitude for display
    phi_c = phi_c / np.max(np.abs(phi_c))

    print("\nExample complex mode shape (normalized, displacement part):")
    print("lambda =", lam_sel)
    print("phi_complex =", phi_c)

    # ------------------------------------------
    # (D) Free response simulation (non-prop)
    # ------------------------------------------
    def zdot(t, z):
        x = z[:n]
        xd = z[n:]
        xdd = np.linalg.solve(M, -C_np @ xd - K @ x)
        return np.hstack([xd, xdd])

    # Initial condition: small displacement in DOF1, rest zero
    x0 = np.array([0.01, 0.0, 0.0])
    v0 = np.zeros(n)
    z0 = np.hstack([x0, v0])

    t_end = 5.0
    sol = solve_ivp(zdot, (0.0, t_end), z0, max_step=1e-2, rtol=1e-8, atol=1e-10)

    # Optional: print a few samples
    print("\nFree response simulation completed.")
    print("t grid size:", sol.t.size)
    print("x(t_end) =", sol.y[:n, -1])

    # If matplotlib is present, plot displacements
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        for i in range(n):
            plt.plot(sol.t, sol.y[i, :], label=f"x{i+1}(t)")
        plt.xlabel("t [s]")
        plt.ylabel("displacement")
        plt.title("Non-proportional damping free response (3-DOF)")
        plt.legend()
        plt.grid(True)
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
