import numpy as np

# Geometry: equilateral base and platform triangles
# Base anchor points B_i in base frame
B = np.array([
    [0.0, 0.0],
    [1.0, 0.0],
    [0.5, np.sqrt(3.0) / 2.0]
])

# Platform anchor points P_i in platform frame (equilateral triangle of radius rp)
rp = 0.2
P = rp * np.array([
    [1.0, 0.0],
    [-0.5, np.sqrt(3.0) / 2.0],
    [-0.5, -np.sqrt(3.0) / 2.0]
])


def rot2(phi):
    """2D rotation matrix."""
    c = np.cos(phi)
    s = np.sin(phi)
    return np.array([[c, -s],
                     [s,  c]])


def ik_3rpr(x, y, phi):
    """
    Inverse kinematics: given planar pose (x, y, phi),
    return leg lengths L (shape (3,)).
    """
    R = rot2(phi)
    t = np.array([x, y])
    L = np.zeros(3)
    for i in range(3):
        ci = t + R @ P[i]       # platform anchor in base frame
        pi = ci - B[i]          # leg vector
        L[i] = np.linalg.norm(pi)
    return L


def fk_residual(z, L):
    """
    Residual vector Phi(z) for FK, where
    z = [x, y, phi], and L is array of leg lengths.
    Phi_i = ||p_i||^2 - L_i^2.
    """
    x, y, phi = z
    R = rot2(phi)
    t = np.array([x, y])
    res = np.zeros(3)
    for i in range(3):
        ci = t + R @ P[i]
        pi = ci - B[i]
        res[i] = pi @ pi - L[i] ** 2
    return res


def fk_jacobian(z):
    """
    Jacobian dPhi/dz for the 3-RPR FK problem.
    z = [x, y, phi].
    """
    x, y, phi = z
    R = rot2(phi)
    # Derivative of R w.r.t. phi
    s = np.sin(phi)
    c = np.cos(phi)
    R_phi = np.array([[-s, -c],
                      [ c, -s]])

    t = np.array([x, y])
    J = np.zeros((3, 3))
    for i in range(3):
        Pi = P[i]
        ci = t + R @ Pi
        pi = ci - B[i]
        # d/dx (||p_i||^2) = 2 p_i_x
        # d/dy (||p_i||^2) = 2 p_i_y
        dphidx = 2.0 * pi[0]
        dphidy = 2.0 * pi[1]
        dpi_dphi = R_phi @ Pi
        dphidphi = 2.0 * (pi @ dpi_dphi)
        J[i, 0] = dphidx
        J[i, 1] = dphidy
        J[i, 2] = dphidphi
    return J


def fk_3rpr_newton(L, z0, tol=1e-10, max_iter=50):
    """
    Newton iteration for FK:
    solve Phi(z) = 0 for z = [x, y, phi].
    L: given leg lengths (shape (3,))
    z0: initial guess (array-like of length 3)
    """
    z = np.array(z0, dtype=float)
    for k in range(max_iter):
        res = fk_residual(z, L)
        norm_res = np.linalg.norm(res)
        if norm_res < tol:
            return z, True, k
        J = fk_jacobian(z)
        try:
            dz = np.linalg.solve(J, -res)
        except np.linalg.LinAlgError:
            return z, False, k
        z = z + dz
    return z, False, max_iter


if __name__ == "__main__":
    # Test round trip FK/IK
    x_true, y_true, phi_true = 0.2, 0.1, 0.3
    L = ik_3rpr(x_true, y_true, phi_true)
    z0 = np.array([0.0, 0.0, 0.0])  # rough initial guess
    z_sol, converged, iters = fk_3rpr_newton(L, z0)
    print("converged:", converged, "in", iters, "iterations")
    print("true pose:", x_true, y_true, phi_true)
    print("FK pose:", z_sol)
      
