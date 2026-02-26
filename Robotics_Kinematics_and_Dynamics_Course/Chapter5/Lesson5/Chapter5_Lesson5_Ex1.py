class RobotPoe:
    """
    Simple serial manipulator model using the PoE formulation.

    Attributes
    ----------
    S_list : (6, n) array
        Columns are screw axes in the space frame.
    M : (4, 4) array
        Home configuration of the end-effector.
    """

    def __init__(self, S_list, M):
        S_list = np.asarray(S_list, dtype=float)
        assert S_list.shape[0] == 6
        self.S_list = S_list
        self.n = S_list.shape[1]
        self.M = np.asarray(M, dtype=float)

    def fk(self, q):
        """
        Compute T(q) = exp(S1 q1) ... exp(Sn qn) M.

        Parameters
        ----------
        q : array_like, shape (n,)
            Joint angles (revolute) or displacements (prismatic).

        Returns
        -------
        T : (4, 4) ndarray
            Homogeneous transform in SE(3).
        """
        q = np.asarray(q, dtype=float).reshape(self.n,)
        T = np.eye(4)
        for i in range(self.n):
            xi = self.S_list[:, i]
            T = T @ exp_twist(xi, q[i])
        return T @ self.M

    def check_se3(self, T, tol=1e-6):
        """
        Check whether T is numerically in SE(3).
        Returns (ok, eps_R, eps_det).
        """
        R = T[:3, :3]
        RtR = R.T @ R
        eps_R = np.linalg.norm(RtR - np.eye(3), ord="fro")
        eps_det = abs(np.linalg.det(R) - 1.0)
        ok = (eps_R < tol) and (eps_det < tol)
        return ok, eps_R, eps_det
      
