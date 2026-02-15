import numpy as np

def hat(p):
    """
    Skew-symmetric matrix for cross product:
    hat(p) @ f == np.cross(p, f)
    """
    px, py, pz = p
    return np.array([
        [0.0, -pz,  py],
        [pz,  0.0, -px],
        [-py, px,  0.0]
    ], dtype=float)

def wrench_transform(R, p, w_B):
    """
    Transform wrench from frame B to frame A, given:
    - R: 3x3 rotation matrix from B to A
    - p: 3x1 translation vector (origin of B expressed in A)
    - w_B: 6x1 wrench in frame B, [f_B; m_B]
    Returns w_A: 6x1 wrench in frame A, [f_A; m_A]
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    p = np.asarray(p, dtype=float).reshape(3,)
    w_B = np.asarray(w_B, dtype=float).reshape(6,)

    f_B = w_B[0:3]
    m_B = w_B[3:6]

    # Force transformation
    f_A = R @ f_B

    # Moment transformation
    m_A = hat(p) @ f_A + R @ m_B

    w_A = np.concatenate((f_A, m_A))
    return w_A

if __name__ == "__main__":
    # Example: same configuration as Section 5
    F = 10.0
    f_B = np.array([0.0, 0.0, F])
    m_B = np.zeros(3)
    w_B = np.concatenate((f_B, m_B))

    R = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0]
    ])
    p = np.array([0.5, 0.0, 0.0])  # 0.5 m along x_A

    w_A = wrench_transform(R, p, w_B)
    print("Wrench in A:", w_A)
      
