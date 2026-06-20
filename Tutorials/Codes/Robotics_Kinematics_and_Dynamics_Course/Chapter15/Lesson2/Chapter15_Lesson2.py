import numpy as np

def mass_matrix(q, m=1.0):
    # q = [x, y]
    return m * np.eye(2)

def constraint_phi(q, l=1.0):
    x, y = q
    return np.array([x**2 + y**2 - l**2])

def constraint_J(q):
    x, y = q
    return np.array([[2.0 * x, 2.0 * y]])  # shape (1,2)

def constraint_Jdot_qdot(q, qdot):
    x, y = q
    xdot, ydot = qdot
    # J = [2x, 2y]; d/dt J = [2xdot, 2ydot]
    return 2.0 * (xdot**2 + ydot**2)  # scalar Jdot * qdot

def unconstrained_forces(q, qdot, m=1.0, g=9.81):
    # conservative gravity force in generalized coordinates:
    # m yddot = -m g + ...
    # We place gravity as a generalized force on the RHS
    return np.array([0.0, -m * g])

def step_constrained_dynamics(q, qdot, tau=None, m=1.0, l=1.0):
    """
    Compute qddot and lambda for one time instant
    using Lagrange multipliers for a point mass on a circle.
    """
    if tau is None:
        tau = np.zeros(2)

    M = mass_matrix(q, m)
    J = constraint_J(q)
    f = unconstrained_forces(q, qdot, m=m)

    # Build augmented system:
    # [ M  -J^T ] [ qddot ] = [ tau + f ]
    # [ J   0   ] [ lambda]   [  gamma  ]
    # where gamma = -Jdot * qdot for time-invariant constraint
    gamma = -constraint_Jdot_qdot(q, qdot)

    A = np.block([
        [M, -J.T],
        [J, np.zeros((1, 1))]
    ])

    rhs = np.concatenate([tau + f, np.array([gamma])])

    sol = np.linalg.solve(A, rhs)
    qddot = sol[0:2]
    lam = sol[2]

    return qddot, lam

# Example usage:
q = np.array([1.0, 0.0])       # starting at (l, 0)
qdot = np.array([0.0, 1.0])    # tangential velocity
qddot, lam = step_constrained_dynamics(q, qdot)
print("qddot =", qddot)
print("lambda =", lam)
      
