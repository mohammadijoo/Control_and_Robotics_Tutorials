import numpy as np

def is_spd(M, tol=1e-9):
    """
    Check if a symmetric matrix M is positive definite via Cholesky.
    """
    if M.shape[0] != M.shape[1]:
        return False
    # Symmetry check
    if not np.allclose(M, M.T, atol=tol):
        return False
    try:
        np.linalg.cholesky(M)
        return True
    except np.linalg.LinAlgError:
        return False


def project_to_spd(M, tol=1e-9):
    """
    Project a symmetric matrix onto the cone of SPD matrices
    by eigenvalue clipping.
    """
    # Symmetrize
    A = 0.5 * (M + M.T)
    w, V = np.linalg.eigh(A)
    w_clipped = np.maximum(w, tol)
    return (V * w_clipped) @ V.T


def skew(v):
    """
    Return the 3x3 skew-symmetric matrix for v in R^3.
    """
    x, y, z = v
    return np.array([[0.0, -z,  y],
                     [z,  0.0, -x],
                     [-y, x,  0.0]])


def spatial_inertia(m, c, I_C):
    """
    Build 6x6 spatial inertia from:
      m  : scalar mass
      c  : 3-vector COM in body frame
      I_C: 3x3 inertia about COM
    """
    c = np.asarray(c).reshape(3,)
    I_C = np.asarray(I_C).reshape(3, 3)
    S = skew(c)
    upper_left = I_C + m * S @ S.T
    upper_right = m * S
    lower_left = m * S.T
    lower_right = m * np.eye(3)
    top = np.hstack((upper_left, upper_right))
    bottom = np.hstack((lower_left, lower_right))
    return np.vstack((top, bottom))


def check_link_inertia(m, c, I_C, tol=1e-9, project=False):
    """
    Check and optionally project a single link's inertial parameters.
    Returns a dict with flags and possibly corrected inertia.
    """
    result = {
        "mass_positive": False,
        "I_C_spd": False,
        "triangle_ok": False,
        "spatial_spd": False,
        "I_C_corrected": None,
    }

    if m <= tol:
        return result  # invalid mass

    # Symmetrize I_C
    I_C_sym = 0.5 * (I_C + I_C.T)

    # SPD check
    if is_spd(I_C_sym, tol=tol):
        I_C_spd = I_C_sym
        result["I_C_spd"] = True
    else:
        if not project:
            return result
        I_C_spd = project_to_spd(I_C_sym, tol=tol)

    # Triangle inequalities in principal frame
    w, _ = np.linalg.eigh(I_C_spd)
    J1, J2, J3 = w  # sorted ascending
    tri_ok = (J1 <= J2 + J3 + tol) and (J2 <= J1 + J3 + tol) and (J3 <= J1 + J2 + tol)

    # Assemble spatial inertia and check SPD
    I_spatial = spatial_inertia(m, c, I_C_spd)
    spatial_ok = is_spd(I_spatial, tol=tol)

    result.update({
        "mass_positive": True,
        "triangle_ok": tri_ok,
        "spatial_spd": spatial_ok,
        "I_C_corrected": I_C_spd
    })
    return result


if __name__ == "__main__":
    # Example link parameters
    m = 5.0
    c = np.array([0.1, -0.05, 0.0])
    I_C = np.array([[0.08, 0.0, 0.0],
                    [0.0,  0.09, 0.0],
                    [0.0,  0.0,  0.05]])

    info = check_link_inertia(m, c, I_C, project=True)
    print(info)

    # In a Pinocchio-based pipeline, you could create pinocchio.Inertia objects
    # from (m, c, I_C_corrected) and rebuild the model before computing H(q).
      
