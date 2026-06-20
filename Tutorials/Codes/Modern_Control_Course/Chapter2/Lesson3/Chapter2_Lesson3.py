import numpy as np
from numpy.linalg import norm

# Example matrix (choose one with real, distinct eigenvalues for clean diagonalization)
A = np.array([[4.0, 1.0, 0.0],
              [1.0, 3.0, 1.0],
              [0.0, 1.0, 2.0]])

# 1) Eigen-decomposition
w, V = np.linalg.eig(A)  # w: eigenvalues, V: eigenvectors (columns)
print("Eigenvalues:", w)
print("Eigenvectors (columns):\n", V)

# 2) Diagonalization check: A ?= V D V^{-1}
D = np.diag(w)
A_recon = V @ D @ np.linalg.inv(V)
print("Reconstruction error ||A - VDV^{-1}||_F =", norm(A - A_recon, ord='fro'))

# 3) Symmetric case: orthogonal diagonalization using eigh (more stable for symmetric A)
ws, Q = np.linalg.eigh(A)  # for symmetric matrices: real eigenvalues, orthonormal eigenvectors
Lambda = np.diag(ws)
A_recon2 = Q @ Lambda @ Q.T
print("Symmetric recon error ||A - QΛQ^T||_F =", norm(A - A_recon2, ord='fro'))

# 4) Power iteration (from scratch) for dominant eigenpair
def power_iteration(A, x0=None, max_iter=2000, tol=1e-10):
    n = A.shape[0]
    if x0 is None:
        x = np.random.randn(n)
    else:
        x = np.array(x0, dtype=float).reshape(n)
    x = x / norm(x)

    lam_old = 0.0
    for k in range(max_iter):
        y = A @ x
        x = y / norm(y)

        # Rayleigh quotient estimate (works best when A is symmetric)
        lam = float(x.T @ (A @ x))

        if abs(lam - lam_old) < tol:
            return lam, x, k+1
        lam_old = lam
    return lam, x, max_iter

lam_hat, v_hat, iters = power_iteration(A)
print("Power iteration: lambda_hat =", lam_hat, "iters =", iters)
print("v_hat =", v_hat)
