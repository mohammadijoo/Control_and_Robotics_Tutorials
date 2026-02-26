import numpy as np

def routh_table(coeffs):
    """
    Build the Routh table for a real-coefficient polynomial.
    coeffs: list or array [a_n, a_{n-1}, ..., a_0]
    Returns a 2D numpy array representing the Routh table.
    """
    coeffs = np.array(coeffs, dtype=float)
    n = len(coeffs) - 1  # polynomial degree
    m = int(np.ceil((n + 1) / 2.0))
    R = np.zeros((n + 1, m))

    # Fill first two rows
    R[0, :len(coeffs[0::2])] = coeffs[0::2]
    R[1, :len(coeffs[1::2])] = coeffs[1::2]

    for i in range(2, n + 1):
        for j in range(0, m - 1):
            a = R[i - 2, 0]
            b = R[i - 2, j + 1]
            c = R[i - 1, 0]
            d = R[i - 1, j + 1]
            if abs(c) < 1e-12:
                # Special handling for zero leading entry (epsilon trick)
                c = 1e-6
            R[i, j] = (c * b - a * d) / c
    return R

def is_stable_routh(coeffs, tol=1e-9):
    """
    Test LHP stability via Routh: all first-column entries positive.
    """
    R = routh_table(coeffs)
    first_col = R[:, 0]
    return np.all(first_col > tol)

def degree_of_stability(coeffs):
    """
    Degree of stability alpha = min_i(-Re(p_i)) if stable, else 0.
    """
    roots = np.roots(coeffs)
    if np.any(np.real(roots) >= 0.0):
        return 0.0
    return float(-np.max(np.real(roots)))

# Example: closed-loop polynomial P(s,K) = s^3 + 5 s^2 + 6 s + K
K = 10.0
den = [1.0, 5.0, 6.0, K]
print("Stable (Routh)?", is_stable_routh(den))
print("Degree of stability alpha =", degree_of_stability(den))

# Robotics context: joint-position loop of a simple motor-driven link
# Approximate plant G(s) = K_m / (J s^2 + B s), with proportional gain Kp
J = 0.01   # kg m^2
B = 0.1    # N m s/rad
K_m = 1.0  # N m/rad
Kp = 50.0  # proportional controller

# Closed-loop characteristic: J s^2 + B s + K_m * Kp = 0
den_joint = [J, B, K_m * Kp]
print("Joint loop stable?", is_stable_routh(den_joint))
print("Joint loop alpha =", degree_of_stability(den_joint))
