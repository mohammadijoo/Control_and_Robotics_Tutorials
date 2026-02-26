import numpy as np

# 3x3 Jacobian example (nearly singular)
J = np.array([[0.1, 0.0, 0.0],
              [0.0, 1.0, 0.999],
              [0.0, 1.0, 1.001]])

cond_J = np.linalg.cond(J)
print("cond(J) =", cond_J)

if cond_J > 1e6:
    print("Warning: Jacobian is ill-conditioned")

# Solve J * dq = v
v = np.array([0.0, 0.0, 1.0])
dq = np.linalg.solve(J, v)
print("dq =", dq)
      
