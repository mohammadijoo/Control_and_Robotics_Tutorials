import numpy as np

class Joint:
    def __init__(self, dof: int, name: str = ""):
        self.dof = dof
        self.name = name

class Mechanism:
    def __init__(self, n_links: int, joints, lambda_dim: int = 6):
        """
        n_links: total links including ground
        joints: iterable of Joint objects
        lambda_dim: 6 for spatial, 3 for planar
        """
        self.n_links = n_links
        self.joints = list(joints)
        self.lambda_dim = lambda_dim

    @property
    def n_joints(self) -> int:
        return len(self.joints)

    def mobility_gruebler(self) -> int:
        lam = self.lambda_dim
        N = self.n_links
        J = self.n_joints
        f_sum = sum(j.dof for j in self.joints)
        return lam * (N - 1 - J) + f_sum

    @staticmethod
    def dof_from_constraints(J_phi: np.ndarray, tol: float = 1e-9) -> int:
        """
        J_phi: m x n constraint Jacobian
        DOF = n - rank(J_phi)
        """
        rank = np.linalg.matrix_rank(J_phi, tol)
        n = J_phi.shape[1]
        return n - rank

# Example: planar 4-bar linkage
joints = [Joint(1, name=f"R{i}") for i in range(4)]
fourbar = Mechanism(n_links=4, joints=joints, lambda_dim=3)
print("4-bar mobility (Gruebler):", fourbar.mobility_gruebler())

# Example: constraint Jacobian for a simple loop q1 + q2 + q3 = constant
J_phi = np.array([[1.0, 1.0, 1.0]])  # 1 x 3, rank = 1
dof = Mechanism.dof_from_constraints(J_phi)
print("DOF from constraints:", dof)
      
