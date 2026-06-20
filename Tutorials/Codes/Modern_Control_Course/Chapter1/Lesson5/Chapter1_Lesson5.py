import numpy as np

# Matrices
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0, 1.0],
              [1.0, 0.0]])
C = np.eye(2)
D = np.zeros((2, 2))

# Eigenvalues of A (internal modes)
lam = np.linalg.eigvals(A)
print("eig(A) =", lam)

# Optional: create a state-space object (requires python-control)
# pip install control
import control as ct
sys = ct.ss(A, B, C, D)
print(sys)
      
