import numpy as np

def jacobian_2r(q, l1, l2):
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)

    J11 = -l1 * s1 - l2 * s12
    J12 = -l2 * s12
    J21 =  l1 * c1 + l2 * c12
    J22 =  l2 * c12
    J = np.array([[J11, J12],
                  [J21, J22]])
    return J

def generalized_torques_from_force(q, l1, l2, f):
    """
    q : array-like, shape (2,)
    f : array-like, shape (2,) planar end-effector force [fx, fy]
    """
    J = jacobian_2r(q, l1, l2)
    f_vec = np.asarray(f).reshape(2,)
    tau = J.T @ f_vec
    return tau

if __name__ == "__main__":
    l1, l2 = 1.0, 0.8
    q  = np.array([0.5, -0.3])  # radians
    f  = np.array([10.0, 5.0])  # N in x-y plane

    tau = generalized_torques_from_force(q, l1, l2, f)
    print("Joint torques:", tau)

    # Numerical check of virtual work invariance:
    # Choose a small virtual joint displacement delta_q
    delta_q = np.array([1e-4, -2e-4])
    J = jacobian_2r(q, l1, l2)
    delta_p = J @ delta_q
    deltaW_task = f @ delta_p
    deltaW_joint = tau @ delta_q
    print("deltaW_task ~", deltaW_task, "deltaW_joint ~", deltaW_joint)
      
