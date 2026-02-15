import numpy as np

def fk_2R(q, l1, l2):
    """
    Planar 2R forward kinematics (position only).
    q: array-like of shape (2,) with [q1, q2]
    """
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def jacobian_2R(q, l1, l2):
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)
    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ])
    return J

def jdot_2R(q, dq, l1, l2):
    q1, q2 = q
    dq1, dq2 = dq
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c12 = np.cos(q1 + q2)
    s12 = np.sin(q1 + q2)
    # Derived symbolically in Section 3
    Jdot = np.array([
        [-dq1 * l1 * c1 - (dq1 + dq2) * l2 * c12,
         -(dq1 + dq2) * l2 * c12],
        [-dq1 * l1 * s1 - (dq1 + dq2) * l2 * s12,
         -(dq1 + dq2) * l2 * s12]
    ])
    return Jdot

def task_velocity_acceleration_2R(q, dq, ddq, l1, l2):
    J = jacobian_2R(q, l1, l2)
    Jdot = jdot_2R(q, dq, l1, l2)
    dq = np.asarray(dq).reshape(2,)
    ddq = np.asarray(ddq).reshape(2,)
    xdot = J @ dq
    xddot = J @ ddq + Jdot @ dq
    return xdot, xddot

if __name__ == "__main__":
    l1, l2 = 1.0, 0.7
    q = np.deg2rad([30.0, 20.0])
    dq = np.array([0.5, -0.3])
    ddq = np.array([0.2, 0.1])

    x = fk_2R(q, l1, l2)
    xdot, xddot = task_velocity_acceleration_2R(q, dq, ddq, l1, l2)

    print("End-effector position:", x)
    print("End-effector velocity:", xdot)
    print("End-effector acceleration:", xddot)
      
