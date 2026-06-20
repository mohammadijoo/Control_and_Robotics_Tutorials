
import numpy as np

def forward_kinematics_2d(q, l1, l2):
    """
    Simple 2-DOF planar arm in R^2.
    q: array-like, shape (2,)
    l1, l2: link lengths
    """
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def joint_limit_barrier(q, q_min, q_max, margin=0.0):
    """
    Elementwise joint limit barrier:
    h_low[i] = q[i] - (q_min[i] + margin)
    h_up[i]  = (q_max[i] - margin) - q[i]
    Returns concatenated h vector.
    """
    q = np.asarray(q)
    q_min = np.asarray(q_min)
    q_max = np.asarray(q_max)
    h_low = q - (q_min + margin)
    h_up = (q_max - margin) - q
    return np.concatenate([h_low, h_up])

def obstacle_barrier(q, l1, l2, c, r_obs, d_min):
    """
    Obstacle avoidance barrier:
    h_obs(q) = ||p(q) - c||^2 - (r_obs + d_min)^2
    """
    p = forward_kinematics_2d(q, l1, l2)
    c = np.asarray(c)
    dist2 = np.dot(p - c, p - c)
    rho = r_obs + d_min
    return dist2 - rho**2

def is_safe(q, q_min, q_max, l1, l2, c, r_obs, d_min, margin=0.0, tol=0.0):
    """
    Check whether state q satisfies all constraints:
    joint limits and obstacle avoidance.
    """
    h_joints = joint_limit_barrier(q, q_min, q_max, margin=margin)
    h_obs = obstacle_barrier(q, l1, l2, c, r_obs, d_min)
    # all constraints must be nonnegative (with tolerance)
    return np.all(h_joints >= -tol) and (h_obs >= -tol)

# Example usage:
if __name__ == "__main__":
    q = np.array([0.0, 0.0])
    q_min = np.array([-1.0, -1.0])
    q_max = np.array([1.0, 1.0])
    l1 = 0.8
    l2 = 0.6
    c = np.array([0.8, 0.0])
    r_obs = 0.1
    d_min = 0.05

    print("Joint barrier:", joint_limit_barrier(q, q_min, q_max))
    print("Obstacle barrier:", obstacle_barrier(q, l1, l2, c, r_obs, d_min))
    print("Is safe?", is_safe(q, q_min, q_max, l1, l2, c, r_obs, d_min))
