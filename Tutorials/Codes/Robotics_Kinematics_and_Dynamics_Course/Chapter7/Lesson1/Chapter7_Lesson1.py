import numpy as np

def skew(w):
    """
    w: shape (3,), returns 3x3 skew matrix.
    """
    wx, wy, wz = w
    return np.array([
        [0.0, -wz,  wy],
        [wz,  0.0, -wx],
        [-wy, wx,  0.0]
    ])

def adjoint(T):
    """
    T: 4x4 homogeneous transform.
    returns 6x6 adjoint matrix Ad_T.
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_hat = skew(p)
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3:6, 0:3] = p_hat @ R
    Ad[3:6, 3:6] = R
    return Ad

def forward_velocity_chain(T_list, S_list, qdot, V0=None):
    """
    Velocity propagation for a serial chain.

    Parameters
    ----------
    T_list : list of 4x4 numpy arrays
        T_list[i] is T_{i-1,i}(q_i), the transform from frame i to i-1.
    S_list : list of 6x1 numpy arrays
        Joint screw axes S_i expressed in frame i.
    qdot : numpy array of shape (n,)
        Joint velocities qdot_i.
    V0 : 6x1 numpy array or None
        Twist of the base frame expressed in frame 0. If None, assumes zero.

    Returns
    -------
    V_links : list of 6x1 numpy arrays
        V_links[i] is the twist of link i+1 expressed in frame i+1.
    """
    n = len(S_list)
    assert len(T_list) == n
    assert qdot.shape[0] == n

    if V0 is None:
        V_prev = np.zeros((6, 1))
    else:
        V_prev = np.asarray(V0).reshape(6, 1)

    V_links = []
    for i in range(n):
        T_i = T_list[i]
        Ad_inv = adjoint(np.linalg.inv(T_i))
        V_prev_in_i = Ad_inv @ V_prev
        S_i = S_list[i].reshape(6, 1)
        V_i = V_prev_in_i + S_i * qdot[i]
        V_links.append(V_i)
        V_prev = V_i
    return V_links

# Example usage for planar 2R (both joints about z axis)
L1, L2 = 1.0, 0.8
q = np.array([0.3, 0.5])
qdot = np.array([0.4, 0.2])

# Define relative transforms T_{0,1}(q1), T_{1,2}(q2) numerically
def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    T = np.eye(4)
    T[0:3, 0:3] = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ])
    return T

T01 = rot_z(q[0])
T01[0, 3] = 0.0
T01[1, 3] = 0.0
T01[2, 3] = 0.0

T12 = rot_z(q[1])
T12[0, 3] = L1
T12[1, 3] = 0.0
T12[2, 3] = 0.0

# Joint screws expressed in local joint frames
S1 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
S2 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

V_links = forward_velocity_chain([T01, T12], [S1, S2], qdot)
V1, V2 = V_links

print("Link 1 twist:", V1.flatten())
print("End-effector twist:", V2.flatten())
      
