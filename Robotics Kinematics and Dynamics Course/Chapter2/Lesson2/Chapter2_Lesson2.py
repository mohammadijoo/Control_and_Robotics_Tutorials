import numpy as np

def rot_x(theta: float) -> np.ndarray:
    """Rotation about x-axis by angle theta (radians)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0,  c,  -s ],
                     [0.0,  s,   c ]], dtype=float)

def rot_y(theta: float) -> np.ndarray:
    """Rotation about y-axis by angle theta (radians)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[  c, 0.0,  s ],
                     [0.0, 1.0, 0.0],
                     [ -s, 0.0,  c ]], dtype=float)

def rot_z(theta: float) -> np.ndarray:
    """Rotation about z-axis by angle theta (radians)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, -s, 0.0],
                     [ s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

def is_so3(R: np.ndarray, tol: float = 1e-9) -> bool:
    """Return True if R is in SO(3) within tolerance tol."""
    if R.shape != (3, 3):
        return False
    I = np.eye(3)
    should_be_I = R.T @ R
    orth_err = np.linalg.norm(should_be_I - I, ord="fro")
    det_R = np.linalg.det(R)
    return orth_err < tol and abs(det_R - 1.0) < tol

if __name__ == "__main__":
    theta_deg = 30.0
    theta = np.deg2rad(theta_deg)

    # Compose a rotation: first about z, then about y
    Rz = rot_z(theta)
    Ry = rot_y(theta)
    R = Rz @ Ry

    print("R =\n", R)
    print("R^T R =\n", R.T @ R)
    print("det(R) =", np.linalg.det(R))
    print("R in SO(3)?", is_so3(R))

    # Example: transform a vector between frames
    p_B = np.array([1.0, 0.0, 0.0])  # coordinates in frame B
    p_A = R @ p_B                    # coordinates of same vector in frame A
    print("p_A =", p_A)

    # If spatialmath-python is installed:
    try:
        from spatialmath import SO3
        R_sm = SO3.Rz(theta_deg, unit="deg") * SO3.Ry(theta_deg, unit="deg")
        print("SpatialMath R matrix:\n", R_sm.R)
        print("SpatialMath R in SO(3)?", R_sm.isSO())
    except ImportError:
        print("spatialmath not installed; skipping SO3 demo.")
      
