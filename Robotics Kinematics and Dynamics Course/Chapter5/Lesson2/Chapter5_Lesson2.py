import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    Standard DH homogeneous transform for link i.
    All angles in radians.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,      d   ],
        [0.0,    0.0,     0.0,     1.0  ]
    ])
    return T

def fk_dh(dh_params):
    """
    dh_params: list of (theta, d, a, alpha) for i = 1,...,n.
    Returns T_0_n as 4x4 numpy array.
    """
    T = np.eye(4)
    for (theta, d, a, alpha) in dh_params:
        T = T @ dh_transform(theta, d, a, alpha)
    return T

# Example: planar 2R
l1, l2 = 1.0, 0.7
q1, q2 = 0.5, -0.3

dh_table = [
    (q1, 0.0, l1, 0.0),
    (q2, 0.0, l2, 0.0),
]

T_0_2 = fk_dh(dh_table)
print("T_0_2 =\n", T_0_2)
print("End-effector position:", T_0_2[0:3, 3])
      
