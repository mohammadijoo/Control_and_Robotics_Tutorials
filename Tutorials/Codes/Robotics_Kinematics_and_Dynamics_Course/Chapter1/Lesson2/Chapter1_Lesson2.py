import numpy as np

# Example: 3x3 symmetric inertia-like matrix (kg*m^2)
I = np.array([[0.20, 0.01, 0.00],
              [0.01, 0.15, 0.00],
              [0.00, 0.00, 0.10]])

# Use eigh for symmetric (Hermitian) matrices: more stable than eig
eigvals, eigvecs = np.linalg.eigh(I)

print("Eigenvalues (principal moments):")
print(eigvals)
print("Eigenvectors (principal axes as columns):")
print(eigvecs)

# Verify eigen-equation A v = lambda v numerically
for i in range(len(eigvals)):
    lam = eigvals[i]
    v = eigvecs[:, i]
    lhs = I @ v
    rhs = lam * v
    print(f"Mode {i}: ||A v - lambda v|| = {np.linalg.norm(lhs - rhs):.3e}")
      
