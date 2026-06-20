import numpy as np

def make_transform(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    Construct a homogeneous transform T from rotation R and translation p.

    Parameters
    ----------
    R : (3, 3) ndarray
        Rotation matrix in SO(3).
    p : (3,) ndarray
        Translation vector.

    Returns
    -------
    T : (4, 4) ndarray
        Homogeneous transformation matrix in SE(3).
    """
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p.reshape(3)
    return T

def is_SE3(T: np.ndarray, tol: float = 1e-9) -> bool:
    """
    Check whether T is numerically close to an element of SE(3).
    """
    if T.shape != (4, 4):
        return False

    R = T[0:3, 0:3]
    last_row = T[3, :]

    cond_last = np.allclose(
        last_row, np.array([0.0, 0.0, 0.0, 1.0]), atol=tol
    )
    cond_orth = np.allclose(R.T @ R, np.eye(3), atol=tol)
    cond_det = abs(np.linalg.det(R) - 1.0) < tol

    return bool(cond_last and cond_orth and cond_det)

def inverse_transform(T: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of a homogeneous transform T.
    """
    assert is_SE3(T), "T must be in SE(3)"
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3] = -R.T @ p
    return T_inv

def transform_point(T: np.ndarray, x: np.ndarray) -> np.ndarray:
    """
    Apply T to a point x expressed in the source frame.

    Parameters
    ----------
    T : (4, 4) ndarray
        Homogeneous transform from frame B to frame A.
    x : (3,) ndarray
        Coordinates of the point in frame B.

    Returns
    -------
    y : (3,) ndarray
        Coordinates of the point in frame A.
    """
    x_h = np.hstack((x.reshape(3), 1.0))
    y_h = T @ x_h
    return y_h[0:3]

# Example: chain A->B and B->C to get A->C
R_AB = np.eye(3)
p_AB = np.array([0.5, 0.0, 0.0])

theta = np.deg2rad(90.0)
R_BC = np.array([
    [np.cos(theta), -np.sin(theta), 0.0],
    [np.sin(theta),  np.cos(theta), 0.0],
    [0.0,            0.0,           1.0],
])
p_BC = np.array([0.0, 0.2, 0.0])

T_AB = make_transform(R_AB, p_AB)
T_BC = make_transform(R_BC, p_BC)
T_AC = T_AB @ T_BC

x_C = np.array([0.1, 0.0, 0.0])  # point in frame C
x_A = transform_point(T_AC, x_C)
print("Point in frame A:", x_A)
      
