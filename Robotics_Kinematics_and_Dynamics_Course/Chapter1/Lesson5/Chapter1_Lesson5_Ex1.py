import numpy as np

def fd_jacobian_central(f, q, h=1e-6):
    """
    Central-difference Jacobian of f: R^n -> R^m at point q.
    f(q) must return a 1D array of length m.
    """
    q = np.asarray(q, dtype=float)
    m = len(f(q))
    n = len(q)
    J = np.zeros((m, n))
    for j in range(n):
        dq = np.zeros_like(q)
        dq[j] = h
        f_plus = f(q + dq)
        f_minus = f(q - dq)
        J[:, j] = (f_plus - f_minus) / (2.0 * h)
    return J
      
