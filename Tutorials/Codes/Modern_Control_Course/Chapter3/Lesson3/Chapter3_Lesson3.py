import numpy as np

def expm_series(A, t=1.0, tol=1e-12, max_terms=200):
    """
    Compute exp(A t) via truncated power series.
    Terminates when ||term||_F <= tol * ||S||_F or when max_terms is reached.
    """
    A = np.array(A, dtype=float)
    n = A.shape[0]
    S = np.eye(n)
    term = np.eye(n)
    At = A * float(t)

    for k in range(1, max_terms + 1):
        term = term @ At / float(k)
        S_new = S + term
        # Frobenius norm stopping criterion
        if np.linalg.norm(term, ord='fro') <= tol * np.linalg.norm(S_new, ord='fro'):
            return S_new, k
        S = S_new

    return S, max_terms

# Example: a simple 2x2 rotation generator
A = np.array([[0.0, 1.0],
              [-1.0, 0.0]])
t = 0.7

S_approx, used = expm_series(A, t=t, tol=1e-14, max_terms=200)
print("Series terms used:", used)
print("Series exp(A t):\n", S_approx)

# Compare with SciPy if available
try:
    from scipy.linalg import expm
    S_lib = expm(A * t)
    print("SciPy expm(A t):\n", S_lib)
    print("Fro error:", np.linalg.norm(S_lib - S_approx, ord='fro'))
except Exception as e:
    print("SciPy not available; series result computed only.")
