import numpy as np

REVOLUTE = 0
PRISMATIC = 1

def wrap_to_pi(angle):
    """Map any angle (rad) to (-pi, pi]."""
    # Using modular arithmetic with numpy
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def config_distance(q1, q2, joint_types, weights=None):
    """
    Compute weighted configuration distance between q1 and q2.

    Parameters
    ----------
    q1, q2 : array-like, shape (n,)
        Joint configurations.
    joint_types : array-like of ints (REVOLUTE or PRISMATIC)
    weights : array-like of shape (n,), optional
        Positive weights. If None, all ones.
    """
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    joint_types = np.asarray(joint_types, dtype=int)

    assert q1.shape == q2.shape
    n = q1.size
    assert joint_types.size == n

    if weights is None:
        weights = np.ones(n, dtype=float)
    else:
        weights = np.asarray(weights, dtype=float)
        assert weights.size == n

    deltas = np.empty(n, dtype=float)
    for i in range(n):
        if joint_types[i] == REVOLUTE:
            delta = q2[i] - q1[i]
            delta = wrap_to_pi(delta)
        else:  # PRISMATIC
            delta = q2[i] - q1[i]
        deltas[i] = weights[i] * delta

    return np.linalg.norm(deltas, ord=2)

if __name__ == "__main__":
    # Example: 7-DOF arm: R-R-P-R-R-R-P
    q1 = [0.0, 0.5, 0.1, -1.0, 0.3, 0.2, 0.05]
    q2 = [np.pi - 0.1, 0.4, 0.15, -0.8, 0.0, 0.0, 0.10]
    joint_types = [REVOLUTE, REVOLUTE, PRISMATIC,
                   REVOLUTE, REVOLUTE, REVOLUTE, PRISMATIC]
    # Example weights: normalize revolute joints by pi, prismatic by 0.5 m
    weights = [1.0 / np.pi, 1.0 / np.pi, 1.0 / 0.5,
               1.0 / np.pi, 1.0 / np.pi, 1.0 / np.pi, 1.0 / 0.5]

    d = config_distance(q1, q2, joint_types, weights)
    print("Configuration distance:", d)
      
