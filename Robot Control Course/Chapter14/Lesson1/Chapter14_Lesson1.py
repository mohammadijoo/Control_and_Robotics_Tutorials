
import numpy as np

def damped_pinv(J: np.ndarray, lam: float = 1e-4) -> np.ndarray:
    """
    Compute damped Moore-Penrose pseudoinverse of J.
    """
    m, n = J.shape
    if m >= n:
        # column-rank scenario
        return np.linalg.inv(J.T @ J + (lam ** 2) * np.eye(n)) @ J.T
    else:
        # row-rank scenario
        return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(m))

def nullspace_projector(J: np.ndarray, lam: float = 1e-4) -> np.ndarray:
    """
    Compute null-space projector N = I - J^# J using damped pseudoinverse.
    """
    n = J.shape[1]
    J_pinv = damped_pinv(J, lam)
    I = np.eye(n)
    return I - J_pinv @ J

def two_task_velocity_control(J1: np.ndarray,
                              xdot1_star: np.ndarray,
                              J2: np.ndarray,
                              xdot2_star: np.ndarray,
                              lam: float = 1e-4) -> np.ndarray:
    """
    Compute joint velocities for two hierarchical tasks:
    Task 1 has higher priority than Task 2.
    """
    # Task 1 solution
    J1_pinv = damped_pinv(J1, lam)
    qdot1 = J1_pinv @ xdot1_star

    # Null-space projector of Task 1
    N1 = nullspace_projector(J1, lam)

    # Residual for Task 2, respecting Task 1
    residual2 = xdot2_star - J2 @ qdot1
    J2_pinv = damped_pinv(J2 @ N1, lam)  # effective Jacobian in null space
    qdot2_correction = N1 @ J2_pinv @ residual2

    qdot = qdot1 + qdot2_correction
    return qdot

# Example usage with dummy Jacobians
if __name__ == "__main__":
    n_dof = 7
    m1 = 3  # task 1: 3D position
    m2 = 3  # task 2: 3D orientation

    # Random but well-conditioned Jacobians (for illustration)
    rng = np.random.default_rng(0)
    J1 = rng.normal(size=(m1, n_dof))
    J2 = rng.normal(size=(m2, n_dof))

    xdot1_star = np.array([0.1, 0.0, -0.05])
    xdot2_star = np.array([0.0, 0.05, 0.0])

    qdot = two_task_velocity_control(J1, xdot1_star, J2, xdot2_star, lam=1e-3)
    print("Joint velocity command:", qdot)
