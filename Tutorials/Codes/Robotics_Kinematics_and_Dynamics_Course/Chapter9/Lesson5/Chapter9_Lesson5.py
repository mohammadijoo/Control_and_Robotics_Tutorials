import numpy as np

def planar2R_kinematics(q, l1, l2):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def planar2R_jacobian(q, l1, l2):
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)
    J = np.zeros((2, 2))
    J[0, 0] = -l1 * s1 - l2 * s12
    J[0, 1] = -l2 * s12
    J[1, 0] =  l1 * c1 + l2 * c12
    J[1, 1] =  l2 * c12
    return J

def planar2R_gravity_torque(q, l1, l2, m1, m2, g=9.81):
    """
    Gravity-only static torques from potential energy.
    Link COMs at l1/2 and l2/2 along each link.
    """
    q1, q2 = q
    # y-coordinates of COMs
    y1 = (l1 / 2.0) * np.sin(q1)
    y2 = l1 * np.sin(q1) + (l2 / 2.0) * np.sin(q1 + q2)
    V = m1 * g * y1 + m2 * g * y2
    # symbolic gradient via finite differences (for clarity)
    eps = 1e-6
    grad = np.zeros(2)
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps
        V_plus = planar2R_potential(q + dq, l1, l2, m1, m2, g)
        V_minus = planar2R_potential(q - dq, l1, l2, m1, m2, g)
        grad[i] = (V_plus - V_minus) / (2.0 * eps)
    return grad

def planar2R_potential(q, l1, l2, m1, m2, g):
    q1, q2 = q
    y1 = (l1 / 2.0) * np.sin(q1)
    y2 = l1 * np.sin(q1) + (l2 / 2.0) * np.sin(q1 + q2)
    return m1 * g * y1 + m2 * g * y2

def constrained_static_torque(q, l1, l2, m1, m2,
                              w_ext, lamb):
    """
    q      : [q1, q2]
    w_ext  : 6D spatial wrench at end-effector (Fx, Fy, 0, 0, 0, 0)
    lamb   : scalar constraint multiplier (normal reaction)
    """
    q = np.asarray(q, dtype=float)
    w_ext = np.asarray(w_ext, dtype=float).reshape(6,)
    # gravity
    g_tau = planar2R_gravity_torque(q, l1, l2, m1, m2)
    # Jacobian for planar forces (2x2)
    Jxy = planar2R_jacobian(q, l1, l2)
    # embed planar forces into 6D: [Fx, Fy, 0, 0, 0, 0]
    J = np.zeros((6, 2))
    J[0:2, :] = Jxy
    # external torque
    tau_ext = J.T @ w_ext
    # constraint Jacobian for phi(q) = x(q) - x0
    # only horizontal component
    J_phi = np.array([[-l1 * np.sin(q[0]) - l2 * np.sin(q[0] + q[1]),
                       -l2 * np.sin(q[0] + q[1])]])
    tau_con = J_phi.T * lamb
    tau_a = g_tau + tau_ext + tau_con
    return tau_a

if __name__ == "__main__":
    l1, l2 = 1.0, 0.7
    m1, m2 = 2.0, 1.0
    q = np.deg2rad([40.0, 30.0])
    w_ext = np.array([0.0, -20.0, 0.0, 0.0, 0.0, 0.0])  # downward 20 N
    lamb = 50.0  # desired normal reaction at wall
    tau = constrained_static_torque(q, l1, l2, m1, m2, w_ext, lamb)
    print("Required joint torques:", tau)

# Note: In a full robotics stack you could instead use
# the 'roboticstoolbox-python' library (Corke) to build
# the 2R manipulator, compute Jacobians and static torques,
# and then add constraint reactions as additional generalized forces.
      
