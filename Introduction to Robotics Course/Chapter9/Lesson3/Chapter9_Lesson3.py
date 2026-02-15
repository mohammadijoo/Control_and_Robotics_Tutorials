import numpy as np

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])

def rigid_apply(R, p, x):
    # x' = R x + p
    return R @ x + p

def rigid_compose(R_cb, p_cb, R_ba, p_ba):
    # R_ca = R_cb R_ba ; p_ca = R_cb p_ba + p_cb
    R_ca = R_cb @ R_ba
    p_ca = R_cb @ p_ba + p_cb
    return R_ca, p_ca

theta = np.pi/4
R = Rz(theta)
p = np.array([0.5, -0.2, 0.1])
xA = np.array([1.0, 0.0, 0.0])

xB = rigid_apply(R, p, xA)
print("xB =", xB)
