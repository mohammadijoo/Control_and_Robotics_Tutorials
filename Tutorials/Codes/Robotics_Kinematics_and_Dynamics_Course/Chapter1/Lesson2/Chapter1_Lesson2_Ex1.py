import numpy as np

def power_iteration(A, max_iters=1000, tol=1e-8):
    n = A.shape[0]
    v = np.random.randn(n)
    v = v / np.linalg.norm(v)
    lam_old = 0.0

    for k in range(max_iters):
        w = A @ v
        v = w / np.linalg.norm(w)
        lam = float(v.T @ (A @ v))  # Rayleigh quotient

        if abs(lam - lam_old) < tol:
            break
        lam_old = lam

    return lam, v

# Example matrix: could be a local linearization of a 2D state update
A = np.array([[0.9, 0.1],
              [0.0, 0.8]])

lam_dom, v_dom = power_iteration(A)
print("Dominant eigenvalue approximation:", lam_dom)
print("Dominant eigenvector approximation:", v_dom)
      
