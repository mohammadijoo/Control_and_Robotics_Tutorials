import numpy as np

def phase_variable_realization(a, b):
    """
    Build (A,B) for y^{(n)} + a[n-1] y^{(n-1)} + ... + a[0] y = sum_j b[j] u_j.
    Inputs:
      a: array-like of length n (a[0]=a0, ..., a[n-1]=a_{n-1})
      b: array-like of shape (m,) or (m,1) for m inputs
    Returns:
      A: (n,n), B: (n,m)
    """
    a = np.asarray(a, dtype=float).reshape(-1)
    n = a.size
    b = np.asarray(b, dtype=float).reshape(-1)
    m = b.size

    A = np.zeros((n, n), dtype=float)
    # shift structure
    for i in range(n - 1):
        A[i, i + 1] = 1.0
    # last row
    A[n - 1, :] = -a  # [-a0, -a1, ..., -a_{n-1}]

    B = np.zeros((n, m), dtype=float)
    B[n - 1, :] = b
    return A, B

# Example 2 parameters: y''' + a2 y'' + a1 y' + a0 y = b1 u1 + b2 u2
a0, a1, a2 = 2.0, 3.0, 1.0
b1, b2 = 5.0, -1.0
A, B = phase_variable_realization([a0, a1, a2], [b1, b2])

# Outputs y1=y, y2=y'
C = np.array([[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0]])
D = np.zeros((2, 2))

print("A=\n", A)
print("B=\n", B)
print("C=\n", C)
print("D=\n", D)

# Optional packaging with python-control (pip install control)
try:
    import control
    sys = control.ss(A, B, C, D)
    print(sys)
except Exception as e:
    print("python-control not available or not installed:", e)
      
