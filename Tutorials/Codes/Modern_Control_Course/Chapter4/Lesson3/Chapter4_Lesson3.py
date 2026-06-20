import numpy as np
from numpy.linalg import matrix_rank
from scipy.linalg import null_space

A = np.array([[0., 1., 0.],
              [0., 0., 0.],
              [0., 0., -1.]])
B = np.array([[0.],
              [1.],
              [0.]])
C = np.array([[1., 0., 0.]])
D = np.array([[0.]])

n = A.shape[0]

# Build O = [C; C A; ...; C A^(n-1)]
O_blocks = []
Ak = np.eye(n)
for k in range(n):
    O_blocks.append(C @ Ak)
    Ak = Ak @ A
O = np.vstack(O_blocks)

print("O =\n", O)
print("rank(O) =", matrix_rank(O))

Ny_basis = null_space(O)  # columns span ker(O)
print("Basis for Ny = ker(O) (columns):\n", Ny_basis)

# Interpretation: any x0 in span(Ny_basis) satisfies C e^{A t} x0 = 0 for all t >= 0.
# Optional: verify numerically at sample times
ts = np.linspace(0, 5, 6)
x0 = Ny_basis[:, [0]]  # pick one basis direction
for t in ts:
    # crude expm via scipy if desired
    from scipy.linalg import expm
    y = C @ expm(A*t) @ x0
    print(f"t={t: .2f}, y(t)={float(y): .3e}")
      
