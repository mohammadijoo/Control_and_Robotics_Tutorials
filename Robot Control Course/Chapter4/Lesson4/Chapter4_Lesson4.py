
import numpy as np

def pseudoinverse(J, damping=0.0):
    """
    Right pseudoinverse J# = J^T (J J^T + λ^2 I)^(-1)
    for full-row-rank Jacobian J (m x n, m <= n).
    """
    J = np.asarray(J, dtype=float)
    m, n = J.shape
    if damping > 0.0:
        JJt = J @ J.T + (damping ** 2) * np.eye(m)
    else:
        JJt = J @ J.T
    return J.T @ np.linalg.inv(JJt)

def two_task_hierarchy(J1, dx1, J2, dx2, damping1=0.0, damping2=0.0):
    """
    Compute joint velocity command dq implementing:
      - Task 1 with highest priority
      - Task 2 projected into nullspace of Task 1.

    J1: m1 x n, primary task Jacobian
    dx1: m1 vector, desired primary task velocity
    J2: m2 x n, secondary task Jacobian
    dx2: m2 vector, desired secondary task velocity
    """
    J1 = np.asarray(J1, dtype=float)
    J2 = np.asarray(J2, dtype=float)
    dx1 = np.asarray(dx1, dtype=float).reshape(-1)
    dx2 = np.asarray(dx2, dtype=float).reshape(-1)

    n = J1.shape[1]
    I = np.eye(n)

    # Primary task
    J1_pinv = pseudoinverse(J1, damping=damping1)
    dq1 = J1_pinv @ dx1
    N1 = I - J1_pinv @ J1  # nullspace projector of task 1

    # Secondary task projected in nullspace of primary
    # Use effective Jacobian in the nullspace
    J2_bar = J2 @ N1
    J2_bar_pinv = pseudoinverse(J2_bar, damping=damping2)
    dx2_tilde = dx2 - J2 @ dq1
    dq2_null = J2_bar_pinv @ dx2_tilde

    dq = dq1 + N1 @ dq2_null
    return dq

# Example usage: a 7-DOF arm with Cartesian task (3D position)
if __name__ == "__main__":
    # Random demo Jacobians (for illustration)
    np.random.seed(0)
    n = 7
    J1 = np.random.randn(3, n)   # primary: Cartesian position
    J2 = np.eye(n)               # secondary: joint posture (identity)

    dx1 = np.array([0.1, -0.05, 0.0])   # desired end-effector velocity
    q_pref = np.zeros(n)                # preferred posture
    q_current = np.array([0.2, -0.3, 0.1, 0.0, 0.2, -0.1, 0.1])

    # Simple posture task: try to move back toward q_pref
    # approximated as dx2 = k_posture * (q_pref - q_current)
    k_posture = 0.5
    dx2 = k_posture * (q_pref - q_current)

    dq_cmd = two_task_hierarchy(J1, dx1, J2, dx2,
                                damping1=1e-3, damping2=1e-3)
    print("Joint velocity command:", dq_cmd)
