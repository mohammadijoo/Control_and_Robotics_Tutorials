# ===== Code block 1 extracted from Chapter7/Lesson1.html =====
import numpy as np
from scipy.linalg import expm, norm

A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
t0 = 0.0
x0 = np.array([1.0, 0.0])

def x_of_t(t: float) -> np.ndarray:
    return expm(A * (t - t0)) @ x0

ts = np.linspace(0.0, 5.0, 6)
X = np.vstack([x_of_t(t) for t in ts])
print("Times:", ts)
print("States:\n", X)
      

# ===== Code block 2 extracted from Chapter7/Lesson1.html =====
import numpy as np

def expm_series(A: np.ndarray, t: float, K: int = 30) -> np.ndarray:
    n = A.shape[0]
    I = np.eye(n)
    At = A * t
    term = I.copy()
    S = I.copy()
    for k in range(1, K + 1):
        term = term @ (At / k)   # term = (At^k)/k!
        S = S + term
    return S

A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
t = 1.0
E_lib = expm(A * t)
E_ser = expm_series(A, t, K=40)
print("Relative error:", np.linalg.norm(E_lib - E_ser) / np.linalg.norm(E_lib))
      

# ===== Code block 3 extracted from Chapter7/Lesson1.html =====
import numpy as np

def expm_via_eig(A: np.ndarray, t: float) -> np.ndarray:
    # Works best if A is diagonalizable and eigenvector matrix is well-conditioned.
    w, V = np.linalg.eig(A)
    Vinv = np.linalg.inv(V)
    return V @ np.diag(np.exp(w * t)) @ Vinv

A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
t = 1.0
E_eig = expm_via_eig(A, t)
E_lib = expm(A * t)
print("Relative error:", np.linalg.norm(E_lib - E_eig) / np.linalg.norm(E_lib))
      
