"""
Chapter23_Lesson4.py
Mapping time-domain specifications to desired closed-loop poles for SISO pole placement.
Self-contained Ackermann implementation using NumPy.
"""

import numpy as np


def damping_ratio_from_overshoot(percent_overshoot: float) -> float:
    """Return zeta from percent overshoot Mp in percent, valid for 0 < Mp < 100."""
    if not (0.0 < percent_overshoot < 100.0):
        raise ValueError("percent_overshoot must be between 0 and 100")
    m = percent_overshoot / 100.0
    L = np.log(m)
    return -L / np.sqrt(np.pi**2 + L**2)


def desired_poles_from_specs(percent_overshoot: float, settling_time: float, settling_fraction: float = 0.02):
    """
    Compute dominant second-order pole pair from percent overshoot and settling time.
    settling_fraction=0.02 uses the 2 percent criterion, approximately Ts = 4/(zeta*wn).
    settling_fraction=0.05 uses the 5 percent criterion, approximately Ts = 3/(zeta*wn).
    """
    zeta = damping_ratio_from_overshoot(percent_overshoot)
    c = 4.0 if abs(settling_fraction - 0.02) < 1e-12 else 3.0
    omega_n = c / (zeta * settling_time)
    sigma = zeta * omega_n
    omega_d = omega_n * np.sqrt(max(0.0, 1.0 - zeta**2))
    return zeta, omega_n, np.array([-sigma + 1j * omega_d, -sigma - 1j * omega_d], dtype=complex)


def companion_extra_poles(dominant_poles, n: int, multiplier: float = 5.0):
    """Place remaining real poles multiplier times farther left than dominant real part."""
    if n < len(dominant_poles):
        raise ValueError("system order n must be at least the number of dominant poles")
    poles = list(dominant_poles)
    sigma = min(-p.real for p in dominant_poles if p.real < 0)
    for i in range(n - len(dominant_poles)):
        poles.append(-(multiplier + i) * sigma)
    return np.array(poles, dtype=complex)


def controllability_matrix(A, b):
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, k) @ b for k in range(n)])


def ackermann_gain(A, b, poles):
    """Compute K such that eig(A - bK) equals poles for a controllable SISO pair."""
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    Wc = controllability_matrix(A, b)
    if np.linalg.matrix_rank(Wc) != n:
        raise ValueError("(A,b) is not controllable")

    coeff = np.poly(poles)  # s^n + a_{n-1}s^{n-1}+...+a0
    phi_A = np.linalg.matrix_power(A, n)
    for i in range(n):
        power = n - 1 - i
        phi_A += coeff[i + 1] * np.linalg.matrix_power(A, power)

    e_nT = np.zeros((1, n))
    e_nT[0, -1] = 1.0
    K = e_nT @ np.linalg.inv(Wc) @ phi_A
    return np.real_if_close(K)


if __name__ == "__main__":
    # Example: third-order SISO plant. Design dominant poles from Mp and Ts.
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [-2.0, -3.0, -1.0]])
    b = np.array([[0.0], [0.0], [1.0]])

    Mp = 10.0       # percent overshoot
    Ts = 2.0        # seconds, 2 percent settling time
    zeta, omega_n, pair = desired_poles_from_specs(Mp, Ts)
    desired = companion_extra_poles(pair, n=A.shape[0], multiplier=6.0)
    K = ackermann_gain(A, b, desired)

    print("zeta =", zeta)
    print("omega_n =", omega_n)
    print("desired poles =", desired)
    print("K =", K)
    print("closed-loop eigenvalues =", np.linalg.eigvals(A - b @ K))
