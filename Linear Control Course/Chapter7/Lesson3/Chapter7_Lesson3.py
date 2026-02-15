import numpy as np

def routh_array(coeffs, eps=1e-9):
    """
    Compute Routh array for a polynomial with real coefficients.

    coeffs: list or 1D array [a_n, ..., a_0], a_n > 0
    returns: R (2D numpy array), n_rhp (number of RHP roots by Routh)
    """
    coeffs = np.array(coeffs, dtype=float)
    n = len(coeffs) - 1  # degree
    m = int(np.ceil((n + 1) / 2.0))
    R = np.zeros((n + 1, m), dtype=float)

    # Fill first two rows: even and odd coefficients
    R[0, :len(coeffs[0::2])] = coeffs[0::2]   # a_n, a_(n-2), ...
    R[1, :len(coeffs[1::2])] = coeffs[1::2]   # a_(n-1), a_(n-3), ...

    # Build subsequent rows
    for i in range(2, n + 1):
        if abs(R[i - 1, 0]) < eps:
            # Special case: zero leading element, use epsilon-perturbation
            R[i - 1, 0] = eps
        for j in range(0, m - 1):
            a = R[i - 1, 0]
            b = R[i - 2, 0]
            c = R[i - 2, j + 1]
            d = R[i - 1, j + 1]
            R[i, j] = (a * c - b * d) / a

    # Count sign changes in first column (ignoring tiny values)
    col = R[:, 0]
    # Replace very small values by eps for sign computation
    col[np.abs(col) < eps] = eps
    signs = np.sign(col)
    sign_changes = 0
    for i in range(len(signs) - 1):
        if signs[i] * signs[i + 1] < 0:
            sign_changes += 1

    return R, sign_changes

# Example: closed-loop for a DC motor joint
import control  # python-control library

K = 10.0
G = control.tf([K], [1, 6, 8, 0])  # G(s) = K / (s(s+2)(s+4))
G_cl = control.feedback(G, 1)      # unity feedback

# Extract characteristic polynomial coefficients
num_cl, den_cl = control.tfdata(G_cl)
den_cl = np.squeeze(den_cl)  # [a_n, ..., a_0]

R, n_rhp = routh_array(den_cl)
print("Routh array:\n", R)
print("Number of RHP roots:", n_rhp)
if n_rhp == 0:
    print("Closed-loop is asymptotically stable.")
else:
    print("Closed-loop is unstable.")
