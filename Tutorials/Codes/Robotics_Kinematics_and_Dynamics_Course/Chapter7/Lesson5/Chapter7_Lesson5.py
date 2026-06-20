import numpy as np

def numeric_jacobian_vec(fk, q, h=1e-6):
    """
    Central-difference Jacobian for f: R^n -> R^m.

    fk : callable, fk(q) -> x (1D array of length m)
    q  : 1D array-like of length n
    h  : stepsize
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    n = q.size
    x0 = np.asarray(fk(q), dtype=float).reshape(-1)
    m = x0.size
    J = np.zeros((m, n))

    for i in range(n):
        dq = np.zeros_like(q)
        dq[i] = h
        xp = np.asarray(fk(q + dq), dtype=float).reshape(-1)
        xm = np.asarray(fk(q - dq), dtype=float).reshape(-1)
        J[:, i] = (xp - xm) / (2.0 * h)

    return J
      
