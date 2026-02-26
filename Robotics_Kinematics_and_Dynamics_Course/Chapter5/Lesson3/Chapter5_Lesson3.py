import numpy as np

def dh_transform(theta, d, a, alpha):
    """Single DH transform using standard convention."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,    sa,       ca,    d    ],
        [0.0,   0.0,      0.0,   1.0   ]
    ])

def fk_dh(q, a, alpha, d, theta_offset=None):
    """
    Generic DH FK.
    q: array of joint variables (n,)
    a, alpha, d: arrays of constant link parameters (n,)
    theta_offset: constant offsets for revolute joints (n,) or None.
    """
    q = np.asarray(q).flatten()
    n = q.shape[0]
    if theta_offset is None:
        theta_offset = np.zeros_like(q)

    T = np.eye(4)
    for i in range(n):
        theta_i = q[i] + theta_offset[i]
        A_i = dh_transform(theta_i, d[i], a[i], alpha[i])
        T = T @ A_i
    return T

# Example: planar 2R with a1=l1, a2=l2, alpha_i=0, d_i=0
def fk_planar_2r(q, l1, l2):
    q = np.asarray(q).flatten()
    a = np.array([l1, l2])
    alpha = np.array([0.0, 0.0])
    d = np.array([0.0, 0.0])
    return fk_dh(q, a, alpha, d)

# Example: planar 3R
def fk_planar_3r(q, l1, l2, l3):
    q = np.asarray(q).flatten()
    a = np.array([l1, l2, l3])
    alpha = np.array([0.0, 0.0, 0.0])
    d = np.array([0.0, 0.0, 0.0])
    return fk_dh(q, a, alpha, d)
      
