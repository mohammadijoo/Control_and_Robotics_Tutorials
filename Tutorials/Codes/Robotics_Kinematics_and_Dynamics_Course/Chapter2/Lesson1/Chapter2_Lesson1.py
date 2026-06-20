import numpy as np

class RigidBodyConfig:
    """
    Represent a rigid body configuration as (R, p),
    where R is a 3x3 rotation matrix and p is a 3D position vector.
    """

    def __init__(self, R, p):
        self.R = np.asarray(R, dtype=float).reshape(3, 3)
        self.p = np.asarray(p, dtype=float).reshape(3)

    def is_valid(self, tol=1e-6):
        """Check R in SO(3) numerically."""
        RT_R = self.R.T @ self.R
        I = np.eye(3)
        cond_orthonormal = np.allclose(RT_R, I, atol=tol)
        cond_det = np.isclose(np.linalg.det(self.R), 1.0, atol=tol)
        return cond_orthonormal and cond_det

# Example: identity pose at the origin
R = np.eye(3)
p = np.zeros(3)
q = RigidBodyConfig(R, p)
print("Valid configuration:", q.is_valid())

# Simple joint-space example: planar 2R joint coordinates
def planar_2R_joint_config(theta1, theta2):
    """
    Return the joint configuration q = [theta1, theta2]^T.
    Forward kinematics mapping to workspace will be introduced later.
    """
    return np.array([theta1, theta2], dtype=float)

q_joint = planar_2R_joint_config(0.5, -0.3)
print("Joint configuration q:", q_joint)
      
