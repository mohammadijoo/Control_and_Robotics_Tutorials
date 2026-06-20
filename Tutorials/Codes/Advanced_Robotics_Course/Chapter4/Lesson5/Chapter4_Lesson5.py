import numpy as np

# Robot parameters: planar 2-DOF arm
L1, L2 = 0.7, 0.7  # link lengths

def fk_end_effector(q):
    """
    Forward kinematics for planar 2R arm.
    q: array-like of shape (2,)
    returns np.array([x, y])
    """
    q1, q2 = q
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    return np.array([x, y])

def jacobian_end_effector(q):
    """
    Geometric Jacobian for the end-effector in 2D.
    """
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)

    # J is 2x2
    J = np.zeros((2, 2))
    J[0, 0] = -L1 * s1 - L2 * s12
    J[0, 1] = -L2 * s12
    J[1, 0] =  L1 * c1 + L2 * c12
    J[1, 1] =  L2 * c12
    return J

# Obstacle: circle centered at (0.5, 0.0) with radius 0.25
obs_center = np.array([0.5, 0.0])
obs_radius = 0.25
clearance = 0.05  # desired clearance

def signed_distance_circle(x):
    """
    Signed distance from a point x in R^2 to a circle.
    Positive outside, negative inside.
    """
    diff = x - obs_center
    norm = np.linalg.norm(diff)
    return norm - obs_radius

def collision_cost_and_grad(path):
    """
    path: ndarray of shape (N+1, 2) with waypoints (including endpoints).
    Returns J_col, grad_col (same shape as path).
    Only internal waypoints are penalized for simplicity.
    """
    N_plus_1, dof = path.shape
    grad = np.zeros_like(path)
    cost = 0.0

    for k in range(1, N_plus_1 - 1):
        qk = path[k]
        xk = fk_end_effector(qk)
        d = signed_distance_circle(xk)
        if d < clearance:
            # hinge penalty phi(d) = 0.5 * (clearance - d)^2
            phi = 0.5 * (clearance - d) ** 2
            cost += phi

            # gradient wrt q_k: phi'(d) * dd/dq
            # phi'(d) = -(clearance - d)
            dphi_dd = -(clearance - d)

            diff = xk - obs_center
            norm = np.linalg.norm(diff)
            if norm > 1e-6:
                n = diff / norm  # workspace normal
                J = jacobian_end_effector(qk)  # 2x2
                # dd/dq = n^T J
                grad_k = dphi_dd * (J.T @ n)
                grad[k] += grad_k
    return cost, grad

def smoothness_cost_and_grad(path):
    """
    Quadratic acceleration penalty using second differences.
    path: ndarray of shape (N+1, dof).
    """
    N_plus_1, dof = path.shape
    grad = np.zeros_like(path)
    cost = 0.0

    for k in range(1, N_plus_1 - 1):
        ddq = path[k + 1] - 2.0 * path[k] + path[k - 1]
        cost += 0.5 * np.dot(ddq, ddq)

        # Gradient contributions:
        grad[k - 1] += -ddq
        grad[k]     +=  2.0 * ddq
        grad[k + 1] += -ddq

    return cost, grad

def project_joint_limits(path, q_min, q_max, q_start, q_goal):
    """
    Clamp each waypoint to joint limits and fix endpoints.
    """
    path[0] = q_start
    path[-1] = q_goal
    path[1:-1] = np.minimum(q_max, np.maximum(q_min, path[1:-1]))
    return path

# Trajectory initialization
N = 40  # number of segments
q_start = np.array([-0.5, 1.0])
q_goal  = np.array([1.0, -0.5])
path = np.linspace(q_start, q_goal, N + 1)

q_min = np.array([-np.pi, -np.pi])
q_max = np.array([ np.pi,  np.pi])

lambda_smooth = 1.0
lambda_col = 10.0
alpha = 0.01

for it in range(200):
    Js, gs = smoothness_cost_and_grad(path)
    Jc, gc = collision_cost_and_grad(path)

    J_total = lambda_smooth * Js + lambda_col * Jc
    grad_total = lambda_smooth * gs + lambda_col * gc

    # Gradient descent on internal nodes only
    path[1:-1] -= alpha * grad_total[1:-1]

    # Enforce limits and boundaries
    path = project_joint_limits(path, q_min, q_max, q_start, q_goal)

    if it % 20 == 0:
        print(f"Iter {it}: J = {J_total:.4f}, J_smooth = {Js:.4f}, J_col = {Jc:.4f}")
      
