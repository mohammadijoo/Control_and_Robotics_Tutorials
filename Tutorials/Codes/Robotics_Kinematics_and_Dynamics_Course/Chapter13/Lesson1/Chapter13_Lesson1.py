import numpy as np

class RigidBodyInertia:
    """
    Rigid body inertial parameters expressed in a given frame B.
    Attributes
    ----------
    m : float
        Mass (kg).
    com : (3,) array_like
        Center of mass position vector c (m), expressed in frame B.
    I_com : (3,3) array_like
        Inertia tensor about the center of mass (kg m^2), expressed in frame B.
    """
    def __init__(self, m, com, I_com):
        self.m = float(m)
        self.com = np.asarray(com, dtype=float).reshape(3)
        self.I_com = np.asarray(I_com, dtype=float).reshape(3, 3)

    @staticmethod
    def skew(v):
        """Return the 3x3 skew-symmetric matrix [v]x for v in R^3."""
        v = np.asarray(v).reshape(3)
        return np.array([[0.0,    -v[2],  v[1]],
                         [v[2],   0.0,   -v[0]],
                         [-v[1],  v[0],  0.0]])

    def inertia_about_point(self, p):
        """
        Inertia tensor about an arbitrary point P, expressed in frame B.
        Parameters
        ----------
        p : (3,) array_like
            Position of P relative to frame origin B (vector Bp).
        Returns
        -------
        I_P : (3,3) ndarray
        """
        p = np.asarray(p, dtype=float).reshape(3)
        # Vector from P to CoM
        c_rel = self.com - p
        # Parallel axis theorem: I_P = I_C + m * (||c_rel||^2 I - c_rel c_rel^T)
        I_C = self.I_com
        c = c_rel
        I3 = np.eye(3)
        I_P = I_C + self.m * ((c @ c) * I3 - np.outer(c, c))
        return I_P

    def rotate(self, R):
        """
        Express inertia parameters in a rotated frame B' that shares the same origin.
        R maps coordinates from B' to B (so that v_B = R v_B').
        """
        R = np.asarray(R, dtype=float).reshape(3, 3)
        # CoM coordinates in B'
        com_Bp = R.T @ self.com
        # Inertia about CoM in new axes: I'_C = R^T I_C R
        I_com_Bp = R.T @ self.I_com @ R
        return RigidBodyInertia(self.m, com_Bp, I_com_Bp)

    def parameter_vector_about_origin(self):
        """
        Return the 10-parameter vector pi_B about the frame B origin.
        """
        # Inertia about frame origin B
        I_B = self.inertia_about_point(np.zeros(3))
        Ixx = I_B[0, 0]
        Iyy = I_B[1, 1]
        Izz = I_B[2, 2]
        Ixy = -I_B[0, 1]
        Ixz = -I_B[0, 2]
        Iyz = -I_B[1, 2]

        mcx = self.m * self.com[0]
        mcy = self.m * self.com[1]
        mcz = self.m * self.com[2]

        pi = np.array([self.m, mcx, mcy, mcz,
                       Ixx, Iyy, Izz, Ixy, Ixz, Iyz], dtype=float)
        return pi

# Example: uniform rod of length L and mass m along the x-axis, CoM at L/2.
def uniform_rod_inertia(m, L):
    """
    Return RigidBodyInertia for a slender rod of length L, mass m, aligned with x-axis.
    Frame B origin at one end of the rod.
    """
    # CoM at (L/2, 0, 0)
    com = np.array([L / 2.0, 0.0, 0.0])

    # Inertia about CoM for slender rod along x: Ixx = 0, Iyy = Izz = (1/12) m L^2
    Ixx_c = 0.0
    Iyy_c = (1.0 / 12.0) * m * L**2
    Izz_c = Iyy_c
    Ixy_c = Ixz_c = Iyz_c = 0.0
    I_com = np.array([[Ixx_c, -Ixy_c, -Ixz_c],
                      [-Ixy_c, Iyy_c, -Iyz_c],
                      [-Ixz_c, -Iyz_c, Izz_c]])

    return RigidBodyInertia(m, com, I_com)

if __name__ == "__main__":
    m = 2.0
    L = 1.0
    rb = uniform_rod_inertia(m, L)
    pi = rb.parameter_vector_about_origin()
    print("Parameter vector pi_B:", pi)
      
