import numpy as np

def jacobian_2r(q, l1, l2):
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)

    J = np.array([
        [-l1*s1 - l2*s12, -l2*s12],
        [ l1*c1 + l2*c12,  l2*c12]
    ])
    return J

def joint_torques_from_force(q, l1, l2, f):
    """
    q  : array-like of shape (2,)  [q1, q2]
    l1,l2 : link lengths
    f  : array-like of shape (2,)  [fx, fy]
    returns tau : np.array shape (2,)
    """
    J = jacobian_2r(q, l1, l2)
    f_vec = np.asarray(f).reshape(2,)
    tau = J.T @ f_vec
    return tau

if __name__ == "__main__":
    q = np.deg2rad([45.0, 30.0])
    l1, l2 = 1.0, 0.8
    f = np.array([10.0, 0.0])  # force in x-direction
    tau = joint_torques_from_force(q, l1, l2, f)
    print("Joint torques:", tau)
      
