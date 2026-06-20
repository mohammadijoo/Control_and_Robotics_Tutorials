import numpy as np

def make_T(R, p):
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def inv_T(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    Tinv = np.eye(4)
    Tinv[0:3, 0:3] = R.T
    Tinv[0:3, 3] = -R.T @ p
    return Tinv

def frob_error(E):
    return np.linalg.norm(E - np.eye(4), ord="fro")

# Example transforms (toy numbers)
R_WB = np.eye(3)
p_WB = np.array([1.0, 0.0, 0.0])
T_WB = make_T(R_WB, p_WB)

R_BC = np.eye(3)
p_BC = np.array([0.0, 2.0, 0.0])
T_BC = make_T(R_BC, p_BC)

# Derived T_WC
T_WC_derived = T_WB @ T_BC

# Direct measurement with slight noise
T_WC_meas = T_WC_derived.copy()
T_WC_meas[0, 3] += 0.05  # small inconsistency

# Cycle error E = T_WB T_BC T_CW_meas
E = T_WB @ T_BC @ inv_T(T_WC_meas)

print("Cycle Frobenius error:", frob_error(E))
