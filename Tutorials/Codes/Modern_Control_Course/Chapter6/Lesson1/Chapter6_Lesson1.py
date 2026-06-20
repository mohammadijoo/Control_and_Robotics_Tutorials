import numpy as np

# Example (SISO, n=2)
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

# 1) Direct evaluation of G(s) = C (sI-A)^{-1} B + D at a chosen s
def G_of_s(s, A, B, C, D):
    n = A.shape[0]
    M = (s * np.eye(n) - A).astype(complex)
    X = np.linalg.solve(M, B.astype(complex))  # solves (sI-A)X = B
    return C.astype(complex) @ X + D.astype(complex)

s = 1j * 2.0
print("G(j2) =", G_of_s(s, A, B, C, D))

# 2) Convert ss -> tf using python-control
# pip install control
import control
sys_ss = control.ss(A, B, C, D)

# Transfer function form (for SISO, yields a scalar tf; for MIMO, a tf matrix)
sys_tf = control.tf(sys_ss)
print(sys_tf)

# 3) Frequency response check
w = np.logspace(-2, 2, 200)
mag, phase, omega = control.bode(sys_ss, w, Plot=False)  # uses stable numerical routines
print("Computed bode arrays:", mag.shape, phase.shape)
      
