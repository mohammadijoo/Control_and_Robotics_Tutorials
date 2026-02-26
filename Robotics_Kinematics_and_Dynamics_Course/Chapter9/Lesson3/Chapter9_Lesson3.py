import numpy as np

def jacobian_2r(q, l1, l2):
    """
    Planar 2R translational Jacobian J_v(q) in the plane.
    q : array-like [q1, q2]
    l1, l2 : link lengths
    """
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

def torque_from_force(q, f, l1=1.0, l2=1.0):
    """
    Compute joint torques tau = J_v(q)^T f for planar 2R arm.
    f : np.array shape (2,) = [Fx, Fy]
    """
    J = jacobian_2r(q, l1, l2)
    tau = J.T @ f
    return tau

if __name__ == "__main__":
    # Example: l1=l2=1, q=[0,0], force downward Fy = -10
    q = np.array([0.0, 0.0])
    f = np.array([0.0, -10.0])
    tau = torque_from_force(q, f, l1=1.0, l2=1.0)
    print("Joint torques:", tau)

    # With roboticstoolbox, for a general robot one could do:
    # from roboticstoolbox import DHRobot, RevoluteDH
    # robot = DHRobot([...])  # define links
    # J_full = robot.jacob0(q)    # 6 x n Jacobian in base frame
    # F6 = np.array([Nx, Ny, Nz, Fx, Fy, Fz])
    # tau_full = J_full.T @ F6
      
