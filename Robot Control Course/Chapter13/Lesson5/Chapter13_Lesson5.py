
import numpy as np

# Robot parameters for planar 2-DOF arm (for illustration)
L1, L2 = 1.0, 1.0  # link lengths

def forward_kinematics(q):
    q1, q2 = q
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    return np.array([x, y])

def jacobian(q):
    q1, q2 = q
    s1 = np.sin(q1); c1 = np.cos(q1)
    s12 = np.sin(q1 + q2); c12 = np.cos(q1 + q2)
    J11 = -L1 * s1 - L2 * s12
    J12 = -L2 * s12
    J21 =  L1 * c1 + L2 * c12
    J22 =  L2 * c12
    return np.array([[J11, J12],
                     [J21, J22]])

# Joint limits
q_min = np.array([-np.pi / 2.0, -np.pi / 2.0])
q_max = np.array([ np.pi / 2.0,  np.pi / 2.0])

# Workspace forbidden circle
c_obs = np.array([0.5, 0.5])
R_obs = 0.3

gamma_joint = 5.0
gamma_ws = 5.0
lambda_slack = 1e3

def build_cbf_constraints(q):
    """
    Build A, b for inequalities A v <= b.
    Includes:
      - 2 inequalities per joint (min, max)
      - 1 workspace-avoidance inequality
    """
    n = q.shape[0]
    A_list = []
    b_list = []

    # Joint limit constraints
    for i in range(n):
        # h_min = q_i - q_min_i
        h_min = q[i] - q_min[i]
        # e_i^T v + gamma * h_min >= 0
        # Rewrite: -e_i^T v <= gamma * h_min
        e_i = np.zeros(n)
        e_i[i] = 1.0
        A_list.append(-e_i)
        b_list.append(gamma_joint * h_min)

        # h_max = q_max_i - q_i
        h_max = q_max[i] - q[i]
        # -e_i^T v + gamma * h_max >= 0
        # Rewrite: e_i^T v <= gamma * h_max
        A_list.append(e_i)
        b_list.append(gamma_joint * h_max)

    # Workspace avoidance
    p = forward_kinematics(q)
    J = jacobian(q)
    diff = p - c_obs
    h_ws = np.dot(diff, diff) - R_obs**2
    # 2 diff^T J v + gamma_ws * h_ws >= 0
    # Rewrite: -2 diff^T J v <= gamma_ws * h_ws
    a_ws = -2.0 * diff @ J  # row vector
    A_list.append(a_ws)
    b_list.append(gamma_ws * h_ws)

    A = np.vstack(A_list)      # shape (m, n)
    b = np.array(b_list)       # shape (m,)
    return A, b

def safety_filter(q, v_nom, solve_qp):
    """
    Given q and nominal velocity v_nom, solve QP:
      min_v,delta  0.5 ||v - v_nom||^2 + 0.5 * lambda_slack * delta^2
      s.t.         A v <= b + delta
                   delta >= 0
    The solve_qp callback should accept (H, f, A, b, C, d)
    representing:
      min 0.5 x^T H x + f^T x
      s.t. A x <= b, C x = d
    """
    n = v_nom.shape[0]
    A, b = build_cbf_constraints(q)

    # Decision variable x = [v; delta]
    H = np.zeros((n + 1, n + 1))
    # Quadratic term for v
    H[:n, :n] = np.eye(n)
    # Quadratic term for delta
    H[n, n] = lambda_slack

    f = np.zeros(n + 1)
    f[:n] = -v_nom

    # Inequalities: A v - 1 * delta <= b
    m = A.shape[0]
    A_qp = np.zeros((m + 1, n + 1))
    b_qp = np.zeros(m + 1)

    # CBF constraints
    A_qp[:m, :n] = A
    A_qp[:m, n] = -1.0
    b_qp[:m] = b

    # delta >= 0  becomes  -delta <= 0
    A_qp[m, n] = -1.0
    b_qp[m] = 0.0

    # No equality constraints
    C = None
    d = None

    x_opt = solve_qp(H, f, A_qp, b_qp, C, d)
    v_safe = x_opt[:n]
    return v_safe

# Example usage (assuming some solve_qp implementation):
# def solve_qp(H, f, A, b, C, d):
#     ...
# In a control loop:
# while running:
#     q = get_current_joint_positions()
#     v_nom = nominal_controller(q, q_des, qdot_des)
#     v_safe = safety_filter(q, v_nom, solve_qp)
#     send_velocity_command(v_safe)
