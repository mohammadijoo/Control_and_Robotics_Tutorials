import numpy as np

class StewartPlatform:
    """
    Simple geometric model of a 6-6 Stewart platform.

    base_points:    array-like, shape (6, 3) with Bi in base frame
    platform_points: array-like, shape (6, 3) with Pi in platform frame
    """
    def __init__(self, base_points, platform_points):
        self.B = np.asarray(base_points, dtype=float).reshape(6, 3)
        self.P = np.asarray(platform_points, dtype=float).reshape(6, 3)

    @staticmethod
    def rot_zyx(phi, theta, psi):
        """Rotation matrix R = Rz(psi) * Ry(theta) * Rx(phi)."""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        Rx = np.array([[1.0, 0.0, 0.0],
                       [0.0, cphi, -sphi],
                       [0.0, sphi, cphi]])
        Ry = np.array([[cth, 0.0, sth],
                       [0.0, 1.0, 0.0],
                       [-sth, 0.0, cth]])
        Rz = np.array([[cpsi, -spsi, 0.0],
                       [spsi, cpsi, 0.0],
                       [0.0, 0.0, 1.0]])
        return Rz @ Ry @ Rx

    def leg_vectors(self, p, R):
        """
        Compute leg vectors di, lengths Li, and unit vectors ui.

        Returns:
            d (6,3), L (6,), u (6,3)
        """
        p = np.asarray(p).reshape(3)
        R = np.asarray(R).reshape(3, 3)
        d = p + (R @ self.P.T).T - self.B  # di = p + R Pi - Bi
        L = np.linalg.norm(d, axis=1)
        u = d / L[:, None]
        return d, L, u

    def inverse_kinematics(self, pose):
        """
        IK: given pose x = [px, py, pz, phi, theta, psi],
        compute leg lengths L.
        """
        px, py, pz, phi, theta, psi = pose
        p = np.array([px, py, pz])
        R = self.rot_zyx(phi, theta, psi)
        _, L, _ = self.leg_vectors(p, R)
        return L

    def jacobian_length_rate(self, pose):
        """
        Compute J_L mapping platform twist t = [v; omega] to leg rates Ldot.
        """
        px, py, pz, phi, theta, psi = pose
        p = np.array([px, py, pz])
        R = self.rot_zyx(phi, theta, psi)
        d, L, u = self.leg_vectors(p, R)

        JL = np.zeros((6, 6))
        for i in range(6):
            ui = u[i, :]
            Rp = R @ self.P[i, :]
            # row: [ui^T, (Rp x ui)^T]
            JL[i, 0:3] = ui
            JL[i, 3:6] = np.cross(Rp, ui)
        return JL

    def forward_kinematics_newton(self, L_target, x0, max_iter=20, tol=1e-8):
        """
        Simple Newton-Raphson forward kinematics using constraint equations.
        pose x = [px, py, pz, phi, theta, psi].
        """
        x = np.array(x0, dtype=float).reshape(6)
        L_target = np.asarray(L_target, dtype=float).reshape(6)

        def constraint(xvec):
            px, py, pz, phi, theta, psi = xvec
            p = np.array([px, py, pz])
            R = self.rot_zyx(phi, theta, psi)
            _, L, _ = self.leg_vectors(p, R)
            # Phi_i = Li^2 - L_target_i^2
            return L**2 - L_target**2

        for k in range(max_iter):
            Phi = constraint(x)
            normPhi = np.linalg.norm(Phi)
            if normPhi < tol:
                break

            # Numerical Jacobian J_F via finite differences
            JF = np.zeros((6, 6))
            eps = 1e-6
            for j in range(6):
                dx = np.zeros(6)
                dx[j] = eps
                Phi_plus = constraint(x + dx)
                JF[:, j] = (Phi_plus - Phi) / eps

            # Newton step: x_{k+1} = x_k - JF^{-1} Phi
            delta = np.linalg.solve(JF, Phi)
            x = x - delta

        return x, normPhi
      
