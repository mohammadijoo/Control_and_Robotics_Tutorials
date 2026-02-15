
import numpy as np

# Dimension of decision vector z (e.g., accelerations + contact forces)
m = 12

# Example Jacobians and desired task values
J1 = np.random.randn(6, m)   # high-priority task (e.g., balance)
b1 = np.zeros(6)
J2 = np.random.randn(3, m)   # lower-priority task (e.g., end-effector)
b2 = np.ones(3) * 0.1

W1 = np.eye(6)
W2 = np.eye(3)
lambda_reg = 1e-4

# -------- Weighted QP (single level with two tasks) --------
alpha1 = 1000.0  # high weight for task 1
alpha2 = 1.0     # lower weight for task 2

H = (alpha1 * J1.T @ W1 @ J1 +
     alpha2 * J2.T @ W2 @ J2 +
     lambda_reg * np.eye(m))
g = -(alpha1 * J1.T @ W1 @ b1 +
      alpha2 * J2.T @ W2 @ b2)

# Assume only simple bound constraints for illustration: lb <= z <= ub
lb = -np.ones(m) * 5.0
ub =  np.ones(m) * 5.0

# Example using qpsolvers interface (install via pip if needed)
from qpsolvers import solve_qp
z_weighted = solve_qp(H, g, A=None, b=None, lb=lb, ub=ub, solver="osqp")

# -------- Approximate hierarchical solution via null-space --------
# Level 1: minimize ||J1 z - b1||^2 + lambda_reg ||z||^2
H1 = J1.T @ J1 + lambda_reg * np.eye(m)
g1 = -J1.T @ b1
z1 = np.linalg.solve(H1, -g1)

# Null-space projector of J1
J1_pinv = np.linalg.pinv(J1, rcond=1e-6)
P1 = np.eye(m) - J1_pinv @ J1

# Level 2: optimize in the null space of task 1:
#   minimize ||J2 (z1 + P1 eta) - b2||^2 w.r.t eta
H2 = P1.T @ J2.T @ J2 @ P1 + lambda_reg * np.eye(m)
g2 = P1.T @ J2.T @ (J2 @ z1 - b2)
eta = np.linalg.solve(H2, -g2)
z_hier = z1 + P1 @ eta

print("Weighted solution residuals:")
print("  Task1:", np.linalg.norm(J1 @ z_weighted - b1))
print("  Task2:", np.linalg.norm(J2 @ z_weighted - b2))

print("Hierarchical solution residuals:")
print("  Task1:", np.linalg.norm(J1 @ z_hier - b1))
print("  Task2:", np.linalg.norm(J2 @ z_hier - b2))
