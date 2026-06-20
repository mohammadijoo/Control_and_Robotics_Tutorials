import numpy as np
from scipy.integrate import solve_ivp

m = 1.0
g = 9.81
L = 1.0
alpha = 5.0   # Baumgarte parameter
beta  = 10.0  # Baumgarte parameter

def dynamics(t, y):
    # State y = [x, y, vx, vy]
    q = y[0:2]
    qdot = y[2:4]

    x, y_pos = q
    vx, vy = qdot

    # Inertia and bias
    M = m * np.eye(2)
    h = np.array([0.0, m * g])

    # Constraint phi(q) = x^2 + y^2 - L^2
    phi = x**2 + y_pos**2 - L**2

    # J_phi and Jdot_phi
    J = np.array([[2.0 * x, 2.0 * y_pos]])   # 1 x 2
    Jdot = np.array([[2.0 * vx, 2.0 * vy]])  # 1 x 2

    # Assemble linear system for [qddot; lambda]
    # K is 3 x 3: [ M   -J^T ]
    #             [ J    0   ]
    K = np.zeros((3, 3))
    K[0:2, 0:2] = M
    K[0:2, 2:3] = -J.T
    K[2:3, 0:2] = J
    # K[2,2] = 0 already

    # Right-hand side
    rhs_dyn = -h
    rhs_con = - (Jdot @ qdot
                 + 2.0 * alpha * (J @ qdot)
                 + beta**2 * phi)
    rhs = np.concatenate((rhs_dyn, rhs_con))

    # Solve for qddot and lambda
    sol = np.linalg.solve(K, rhs)
    qddot = sol[0:2]
    # lam = sol[2]

    return np.concatenate((qdot, qddot))

# Example initial condition slightly off the circle
y0 = np.array([L * 1.01, 0.0, 0.0, 0.0])
t_span = (0.0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], 1001)

sol = solve_ivp(dynamics, t_span, y0, t_eval=t_eval, rtol=1e-6, atol=1e-9)

# Compute constraint violation over time
r2 = sol.y[0, :]**2 + sol.y[1, :]**2
phi_vals = r2 - L**2
print("Max |phi|:", np.max(np.abs(phi_vals)))
      
