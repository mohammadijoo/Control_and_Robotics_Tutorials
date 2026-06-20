
import numpy as np
import cvxpy as cp

# Dimensions
nq = 7       # number of joints
mx = 6       # task-space dimension (e.g. full 6D end-effector twist)

# Example Jacobians and references (in a real system, compute from kinematics)
J_x = np.random.randn(mx, nq)  # end-effector Jacobian
J_p = np.eye(nq)               # "posture" task (identity in joint space)

x_err = np.random.randn(mx)    # current task-space error
xdot_ref = np.zeros(mx)        # desired Cartesian velocity
Kx = 5.0 * np.eye(mx)

qdot_nom = np.zeros(nq)
qdot_ref = np.zeros(nq)
Wp = 0.1 * np.eye(nq)
Wx = np.eye(mx)

# Joint limits on velocity
qdot_min = -0.5 * np.ones(nq)
qdot_max =  0.5 * np.ones(nq)

# Optimization variable
qdot = cp.Variable(nq)

v_x = xdot_ref - Kx @ x_err

# Approximate hierarchical cost:
#   Level 1 (end-effector) with large weight w1
#   Level 2 (posture) with smaller weight w2
w1 = 1000.0
w2 = 1.0
lam_reg = 1e-3

cost_level1 = cp.sum_squares(Wx @ (J_x @ qdot - v_x))
cost_level2 = cp.sum_squares(Wp @ (J_p @ (qdot - qdot_ref)))
cost_reg    = cp.sum_squares(qdot - qdot_nom)

cost = 0.5 * (w1 * cost_level1 + w2 * cost_level2 + lam_reg * cost_reg)

constraints = [
    qdot_min <= qdot,
    qdot     <= qdot_max,
]

prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP)  # or cp.QP, cp.OSQP, cp.ECOS, etc.

print("Status:", prob.status)
print("Optimal qdot:", qdot.value)
