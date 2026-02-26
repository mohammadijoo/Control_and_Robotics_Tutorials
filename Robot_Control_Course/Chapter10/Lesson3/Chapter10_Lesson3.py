
import numpy as np
import cvxpy as cp

# Robot joint parameters
Ts = 0.01  # sampling time
J = 0.05   # joint inertia

A = np.array([[1.0, Ts],
              [0.0, 1.0]])
B = np.array([[0.0],
              [Ts / J]])

# Horizon and cost
N = 20
Q = np.diag([50.0, 1.0])   # penalize position error heavily
R = np.array([[0.1]])

# Joint and torque limits
q_min = -1.5
q_max =  1.5
dq_max = 2.0
tau_max = 2.0

def mpc_step(x0, x_ref):
    """
    Solve constrained MPC step for a single joint.
    x0: current state (2,)
    x_ref: desired joint position (scalar), zero desired velocity.
    """
    nx = A.shape[0]
    nu = B.shape[1]

    # Decision variables
    x = cp.Variable((nx, N + 1))
    u = cp.Variable((nu, N))

    # Cost and constraints
    cost = 0
    constraints = []

    # Initial condition
    constraints.append(x[:, 0] == x0)

    for k in range(N):
        # Stage cost (track reference)
        x_ref_vec = np.array([x_ref, 0.0])
        cost += cp.quad_form(x[:, k] - x_ref_vec, Q) + cp.quad_form(u[:, k], R)

        # Dynamics
        constraints.append(x[:, k + 1] == A @ x[:, k] + B @ u[:, k])

        # State constraints
        constraints.append(x[0, k] >= q_min)
        constraints.append(x[0, k] <= q_max)
        constraints.append(cp.abs(x[1, k]) <= dq_max)

        # Input constraints
        constraints.append(cp.abs(u[:, k]) <= tau_max)

    # Terminal cost
    x_ref_vec = np.array([x_ref, 0.0])
    cost += cp.quad_form(x[:, N] - x_ref_vec, Q)

    # Solve QP
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)

    if prob.status not in ["optimal", "optimal_inaccurate"]:
        raise RuntimeError("MPC QP infeasible or solver failed")

    # Receding-horizon: apply the first control
    u0 = u[:, 0].value
    return u0

# Example usage in a control loop:
if __name__ == "__main__":
    x = np.array([0.0, 0.0])
    x_ref = 1.0
    for k in range(50):
        u0 = mpc_step(x, x_ref)
        # Simulate one step (here using the exact model)
        x = A @ x + B @ u0
        print(f"step {k}, q = {x[0]:.3f}, dq = {x[1]:.3f}, tau = {u0[0]:.3f}")
