import numpy as np

# Physical and numerical parameters
m = 1.0
g = 9.81
R = 1.0
alpha = 10.0    # Baumgarte parameters
beta = 20.0
h = 1e-3        # time step
n_steps = 10000

def phi(q):
    # Constraint: x^2 + y^2 - R^2 = 0
    x, y = q
    return x*x + y*y - R*R

def J(q):
    # Jacobian row (shape 1x2)
    x, y = q
    return np.array([[2.0 * x, 2.0 * y]])

def Jdot(q, v):
    # Time derivative of Jacobian times v uses Jdot as 1x2
    vx, vy = v
    return np.array([[2.0 * vx, 2.0 * vy]])

def step(q, v, h):
    """
    Perform one constrained time step using a symplectic-like update:
      v_{k+1} = v_k + h * vdot_k
      q_{k+1} = q_k + h * v_{k+1}
    """
    # Mass matrix and gravity
    M = m * np.eye(2)
    g_vec = np.array([0.0, m * g])

    J_q = J(q)           # 1x2
    Jdot_q = Jdot(q, v)  # 1x2
    phi_q = phi(q)       # scalar

    # Build 3x3 block matrix A and right-hand side rhs
    A = np.zeros((3, 3))
    # Top-left block: M
    A[0:2, 0:2] = M
    # Top-right block: J^T
    A[0:2, 2] = J_q.T[:, 0]
    # Bottom-left block: J
    A[2, 0:2] = J_q[0, :]

    # Right-hand side
    rhs = np.zeros(3)
    # Dynamics: M vdot = -g + J^T lambda
    rhs[0:2] = -g_vec
    # Constraint acceleration with Baumgarte stabilization
    Jv = J_q.dot(v)[0]                 # scalar
    Jdotv = Jdot_q.dot(v)[0]          # scalar
    rhs[2] = -Jdotv - 2.0 * alpha * Jv - (beta ** 2) * phi_q

    sol = np.linalg.solve(A, rhs)
    vdot = sol[0:2]
    # lambda = sol[2]  # not used further here

    # Symplectic-like update
    v_new = v + h * vdot
    q_new = q + h * v_new

    return q_new, v_new

# Initial conditions: start on circle with small tangential velocity
q = np.array([R, 0.0])
v = np.array([0.0, 0.5])

trajectory_q = []
for k in range(n_steps):
    trajectory_q.append(q.copy())
    q, v = step(q, v, h)

trajectory_q = np.array(trajectory_q)
print("Final constraint value:", phi(q))
      
