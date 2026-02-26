
import numpy as np

# Robot parameters
l1, l2 = 1.0, 0.7
q1_min, q1_max = -np.pi/2, np.pi/2
q2_min, q2_max = -np.pi, np.pi

# Obstacle parameters
x_obs, y_obs = 0.7, 0.3
r_obs = 0.2

def forward_kinematics(q):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def in_joint_limits(q):
    q1, q2 = q
    return (q1_min <= q1 <= q1_max) and (q2_min <= q2 <= q2_max)

def obstacle_constraint(q):
    x, y = forward_kinematics(q)
    dx = x - x_obs
    dy = y - y_obs
    # h(x) = ||x - x_obs||^2 - r_obs^2 >= 0 in free space
    return dx*dx + dy*dy - r_obs**2

def ground_constraint(q):
    # Non-penetration: y(q) >= 0
    _, y = forward_kinematics(q)
    return y  # should be >= 0

def admissible(q):
    """Check if q satisfies joint, workspace, and ground constraints."""
    if not in_joint_limits(q):
        return False
    if obstacle_constraint(q) < 0.0:
        return False
    if ground_constraint(q) < 0.0:
        return False
    return True

if __name__ == "__main__":
    q_test = np.array([0.0, 0.0])
    print("q_test admissible?", admissible(q_test))
