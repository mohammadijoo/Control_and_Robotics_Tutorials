import numpy as np
import scipy.linalg as la

# Optional (if available): pip install control
try:
    import control as ct
except Exception:
    ct = None

A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 1.0]])
D = np.array([[0.0]])

# Poles from A (internal candidate poles)
poles_A = la.eigvals(A)
print("eig(A) =", poles_A)

# SISO transmission zeros via generalized eigenvalues of a Rosenbrock pencil
# P(s) = s*E - F, with
# E = [[I, 0],
#      [0, 0]]
# F = [[A, B],
#      [-C, -D]]
n = A.shape[0]
E = np.block([[np.eye(n), np.zeros((n,1))],
              [np.zeros((1,n)), np.zeros((1,1))]])
F = np.block([[A, B],
              [-C, -D]])

# Generalized eigenvalues solve F v = lambda E v (lambda are finite zeros + possibly inf)
alpha, beta = la.eig(F, E, right=False, left=False, overwrite_a=False, overwrite_b=False, homogeneous_eigvals=True)
# Convert homogeneous eigenvalues alpha/beta to lambda, filtering beta=0 (infinite eigenvalues)
zeros = []
for a, b in zip(alpha, beta):
    if abs(b) > 1e-12:
        zeros.append(a / b)
zeros = np.array(zeros, dtype=complex)
print("zeros (finite generalized eig) =", zeros)

# Validate by forming transfer function (if python-control is installed)
if ct is not None:
    sys = ct.ss(A, B, C, D)
    tf_sys = ct.tf(sys)
    print("Transfer function (unsimplified) =", tf_sys)
    print("Poles from tf =", ct.poles(tf_sys))
    print("Zeros from tf =", ct.zeros(tf_sys))
