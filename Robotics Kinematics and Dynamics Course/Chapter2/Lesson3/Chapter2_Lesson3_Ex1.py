from spatialmath import SE3

T_AB = SE3(R_AB, p_AB)      # construct SE3 from rotation and translation
T_BC = SE3(R_BC, p_BC)
T_AC = T_AB * T_BC          # group multiplication in SE(3)

x_C_h = SE3(0, 0, 0) * 0    # dummy; in practice use vector operations
x_C = np.array([0.1, 0.0, 0.0])
x_A = (T_AC * x_C).A[0:3]   # apply transform, then convert to ndarray
      
