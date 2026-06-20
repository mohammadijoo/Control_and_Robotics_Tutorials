"""
Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
Lesson 5 - Intro to Experimental Modal Analysis and System Identification Concepts

File: Chapter13_Lesson5.py

This script:
1) Simulates a 3-DOF mass-spring-damper system under broadband force excitation.
2) Estimates frequency response functions (FRFs) using the H1 estimator via Welch spectral averages.
3) Computes magnitude-squared coherence.
4) Extracts rough modal estimates (natural frequencies + damping) via peak-picking + half-power bandwidth.
5) Demonstrates a compact Eigensystem Realization Algorithm (ERA) identification from an impulse response.

Dependencies: numpy, scipy, matplotlib
"""

from __future__ import annotations
import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass
import scipy.signal as sig
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


# ----------------------------
# 1) Mechanical model (3-DOF)
# ----------------------------
@dataclass
class MDOF:
    M: NDArray[np.float64]
    C: NDArray[np.float64]
    K: NDArray[np.float64]

    def to_state_space(self):
        n = self.M.shape[0]
        Minv = np.linalg.inv(self.M)
        Z = np.zeros((n, n))
        I = np.eye(n)

        A = np.block([
            [Z, I],
            [-Minv @ self.K, -Minv @ self.C],
        ])

        # input: force vector f(t) in physical coordinates
        B = np.vstack([np.zeros((n, n)), Minv])

        # outputs: displacement and acceleration (choose later)
        Cx = np.block([I, Z])                           # displacement
        Cv = np.block([Z, I])                           # velocity
        Ca = np.block([-Minv @ self.K, -Minv @ self.C])  # acceleration (assuming output = qdd)

        D = np.zeros((n, n))
        return A, B, Cx, Cv, Ca, D


def make_chain_3dof() -> MDOF:
    # masses
    m1, m2, m3 = 1.0, 0.9, 0.8
    M = np.diag([m1, m2, m3])

    # springs: grounded at DOF1 and DOF3, coupling springs between masses
    k1, k2, k3, k4 = 800.0, 600.0, 500.0, 700.0
    # K for chain: (ground)-k1-m1-k2-m2-k3-m3-k4-(ground)
    K = np.array([
        [k1 + k2, -k2,      0.0],
        [-k2,     k2 + k3, -k3 ],
        [0.0,     -k3,     k3 + k4],
    ])

    # viscous damping (Rayleigh-like, but specified directly for simplicity)
    c1, c2, c3, c4 = 2.0, 1.8, 1.5, 2.2
    C = np.array([
        [c1 + c2, -c2,      0.0],
        [-c2,     c2 + c3, -c3 ],
        [0.0,     -c3,     c3 + c4],
    ])
    return MDOF(M=M, C=C, K=K)


# ---------------------------------------------
# 2) Time simulation under broadband excitation
# ---------------------------------------------
def simulate(mdof: MDOF, fs: float, T: float, force_dof: int = 0, seed: int = 7):
    """
    Simulate with band-limited white noise force on one DOF.

    Returns:
        t (N,), u (N,), y (N,) where y is acceleration at a measured DOF (same as force_dof by default).
    """
    rng = np.random.default_rng(seed)
    dt = 1.0 / fs
    t = np.arange(0.0, T, dt)
    N = len(t)

    # force: band-limited white noise (shape through lowpass filter)
    u = rng.normal(0.0, 1.0, size=N)
    b, a = sig.butter(4, 0.25)  # cutoff at 0.25*(Nyquist) => 0.125*fs
    u = sig.filtfilt(b, a, u)
    u = u / np.std(u)

    A, B, Cx, Cv, Ca, D = mdof.to_state_space()
    n = mdof.M.shape[0]

    # Input vector f(t) has dimension n; excite one coordinate
    def rhs(ti, x):
        # interpolate input
        k = int(np.clip(np.floor(ti / dt), 0, N - 1))
        f = np.zeros(n)
        f[force_dof] = u[k]
        return (A @ x + B @ f)

    x0 = np.zeros(2 * n)
    sol = solve_ivp(rhs, (t[0], t[-1]), x0, t_eval=t, method="RK45", rtol=1e-7, atol=1e-9)
    X = sol.y.T  # (N, 2n)

    # acceleration output at measured DOF (here: same DOF as force)
    y = (Ca @ X.T).T[:, force_dof]

    # add mild measurement noise
    y = y + 0.02 * rng.normal(size=N)

    return t, u, y


# ----------------------------------------------------
# 3) FRF estimation: Welch averages + H1 + coherence
# ----------------------------------------------------
def estimate_frf_h1(u: NDArray[np.float64], y: NDArray[np.float64], fs: float,
                    nperseg: int = 4096, noverlap: int = 2048):
    """
    H1 estimator:
        H1(w) = G_yu(w) / G_uu(w)

    Coherence:
        gamma^2(w) = |G_yu(w)|^2 / (G_uu(w) G_yy(w))
    """
    f, G_uu = sig.welch(u, fs=fs, nperseg=nperseg, noverlap=noverlap, return_onesided=True)
    f, G_yy = sig.welch(y, fs=fs, nperseg=nperseg, noverlap=noverlap, return_onesided=True)
    f, G_yu = sig.csd(y, u, fs=fs, nperseg=nperseg, noverlap=noverlap, return_onesided=True)

    H1 = G_yu / G_uu
    coh = np.abs(G_yu) ** 2 / (G_uu * G_yy + 1e-30)
    return f, H1, coh


# ----------------------------------------------------
# 4) Peak-picking + half-power damping (rough)
# ----------------------------------------------------
def half_power_damping(f: NDArray[np.float64], Hmag: NDArray[np.float64], peak_idx: int):
    """
    For an SDOF-like isolated peak:
        zeta ≈ (f2 - f1) / (2 fn)
    where f1 and f2 are the half-power frequencies (|H| drops to 1/sqrt(2) of peak).
    """
    fn = f[peak_idx]
    peak = Hmag[peak_idx]
    target = peak / np.sqrt(2.0)

    # search left
    i1 = peak_idx
    while i1 > 1 and Hmag[i1] > target:
        i1 -= 1
    # linear interpolate
    f1 = np.interp(target, [Hmag[i1], Hmag[i1 + 1]], [f[i1], f[i1 + 1]])

    # search right
    i2 = peak_idx
    while i2 < len(f) - 2 and Hmag[i2] > target:
        i2 += 1
    f2 = np.interp(target, [Hmag[i2 - 1], Hmag[i2]], [f[i2 - 1], f[i2]])

    zeta = (f2 - f1) / (2.0 * fn + 1e-30)
    return fn, zeta, f1, f2


def extract_modes(f: NDArray[np.float64], H1: NDArray[np.complex128], coh: NDArray[np.float64],
                  fmax: float = 80.0, min_coh: float = 0.6):
    """
    Simple peak-picking on |H1|, filtering by coherence.
    """
    mask = (f > 0.5) & (f < fmax) & (coh > min_coh)
    f2 = f[mask]
    Hm = np.abs(H1[mask])

    peaks, props = sig.find_peaks(Hm, prominence=np.percentile(Hm, 70), distance=10)
    est = []
    for p in peaks[:3]:  # expect ~3 modes
        fn, zeta, f1, f2hp = half_power_damping(f2, Hm, p)
        est.append((fn, zeta, f1, f2hp))
    return est


# ----------------------------------------------------
# 5) ERA identification from impulse response (SISO)
# ----------------------------------------------------
def simulate_impulse(mdof: MDOF, fs: float, T: float, in_dof: int = 0, out_dof: int = 0):
    """
    Approximate impulse response by applying a narrow pulse input in continuous time.
    Output is acceleration at out_dof.
    """
    dt = 1.0 / fs
    t = np.arange(0.0, T, dt)
    N = len(t)

    # pulse area ~1 to mimic impulse
    u = np.zeros(N)
    u[0] = 1.0 / dt

    A, B, Cx, Cv, Ca, D = mdof.to_state_space()
    n = mdof.M.shape[0]

    def rhs(ti, x):
        k = int(np.clip(np.floor(ti / dt), 0, N - 1))
        f = np.zeros(n)
        f[in_dof] = u[k]
        return (A @ x + B @ f)

    x0 = np.zeros(2 * n)
    sol = solve_ivp(rhs, (t[0], t[-1]), x0, t_eval=t, method="RK45", rtol=1e-8, atol=1e-10)
    X = sol.y.T
    y = (Ca @ X.T).T[:, out_dof]
    return t, y


def era_siso(y: NDArray[np.float64], s: int, r: int, order: int):
    """
    Eigensystem Realization Algorithm (ERA) for SISO impulse response.

    Inputs:
        y[k] = Markov parameters (impulse response samples), k=0..N-1
        s, r : Hankel block sizes
        order: desired reduced order (model order)

    Returns:
        A, B, C (discrete-time), and singular values.
    """
    # Build Hankel matrices H0 and H1
    # H0[i,j] = y[i+j], H1[i,j] = y[i+j+1]
    H0 = np.zeros((s, r))
    H1 = np.zeros((s, r))
    for i in range(s):
        for j in range(r):
            H0[i, j] = y[i + j]
            H1[i, j] = y[i + j + 1]

    U, S, Vt = np.linalg.svd(H0, full_matrices=False)
    U1 = U[:, :order]
    S1 = np.diag(S[:order])
    V1 = Vt[:order, :]

    S1_sqrt = np.sqrt(S1)
    S1_isqrt = np.linalg.inv(S1_sqrt)

    A = S1_isqrt @ (U1.T @ H1 @ V1.T) @ S1_isqrt
    B = S1_sqrt @ V1[:, [0]]    # first column corresponds to first Markov parameter
    C = U1[[0], :] @ S1_sqrt    # first row corresponds to first output
    return A, B, C, S


def main():
    fs = 500.0   # Hz
    T = 40.0     # seconds
    mdof = make_chain_3dof()

    t, u, y = simulate(mdof, fs=fs, T=T, force_dof=0, seed=7)
    f, H1, coh = estimate_frf_h1(u, y, fs=fs, nperseg=4096, noverlap=2048)

    # Extract rough modal estimates
    est = extract_modes(f, H1, coh, fmax=80.0, min_coh=0.6)
    print("Rough modal estimates (fn Hz, zeta, half-power f1,f2):")
    for (fn, zeta, f1, f2hp) in est:
        print(f"  fn={fn:7.3f} Hz, zeta={zeta:7.4f}, f1={f1:7.3f}, f2={f2hp:7.3f}")

    # Plots
    plt.figure()
    plt.semilogy(f, np.abs(H1))
    plt.xlim(0, 120)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("|H1(jw)| [acc/force]")
    plt.title("Estimated FRF (H1)")

    plt.figure()
    plt.plot(f, coh)
    plt.ylim(0, 1.05)
    plt.xlim(0, 120)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Coherence")
    plt.title("Magnitude-squared coherence")

    # ERA from impulse response (SISO)
    t_imp, y_imp = simulate_impulse(mdof, fs=fs, T=10.0, in_dof=0, out_dof=0)
    s = 150
    r = 150
    order = 6  # 2n states for 3-DOF, but measured accel is SISO; try reduced order
    A, B, C, Svals = era_siso(y_imp, s=s, r=r, order=order)

    eigvals = np.linalg.eigvals(A)
    # Convert discrete poles to continuous (approx): lambda_c = ln(z)/dt
    dt = 1.0 / fs
    lambdas = np.log(eigvals) / dt
    wn = np.abs(lambdas) / (2.0 * np.pi)
    zeta = -np.real(lambdas) / (np.abs(lambdas) + 1e-30)

    print("\nERA (discrete-time) identified poles (continuous approx):")
    for i in range(len(lambdas)):
        print(f"  pole {i+1}: lambda={lambdas[i]: .4e}, fn~{wn[i]:.3f} Hz, zeta~{zeta[i]:.4f}")

    plt.figure()
    plt.semilogy(Svals, marker="o")
    plt.xlabel("Index")
    plt.ylabel("Singular value")
    plt.title("ERA Hankel singular values")

    plt.show()


if __name__ == "__main__":
    main()
