import numpy as np

def skew(v):
    """Skew-symmetric matrix [v]x for v in R^3."""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2],  0.0, -v[0]],
                     [-v[1], v[0], 0.0]])

def exp_se3(xi, theta):
    """
    Exponential map for a screw axis xi in R^6 and scalar theta.
    xi = [omega; v]. Uses standard SE(3) matrix exponential.
    """
    omega = xi[0:3]
    v = xi[3:6]
    w_norm = np.linalg.norm(omega)
    if w_norm < 1e-9:
        R = np.eye(3)
        p = v * theta
    else:
        wn = omega / w_norm
        wx = skew(wn)
        R = (np.eye(3)
             + np.sin(w_norm * theta) * wx
             + (1.0 - np.cos(w_norm * theta)) * (wx @ wx))
        V = (np.eye(3) * theta
             + (1.0 - np.cos(w_norm * theta)) / w_norm * wx
             + (w_norm * theta - np.sin(w_norm * theta)) / (w_norm ** 2) * (wx @ wx))
        p = V @ v
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def adjoint(T):
    """Spatial adjoint transform (6x6) of homogeneous transform T."""
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3:6, 3:6] = R
    Ad[3:6, 0:3] = skew(p) @ R
    return Ad

class Link:
    def __init__(self, parent, S, T_parent_to_i0):
        """
        parent: integer parent index
        S: 6x1 motion subspace vector of the joint
        T_parent_to_i0: 4x4 transform at q_i = 0
        """
        self.parent = parent
        self.S = S.reshape(6, 1)
        self.T_parent_to_i0 = T_parent_to_i0

class KinematicTree:
    def __init__(self, links):
        """
        links: list of Link objects, index 0 reserved for floating base pseudo-link.
        """
        self.links = links
        self.N = len(links) - 1  # number of actuated joints/links except base

    def forward_kinematics(self, q_base, v_base, q):
        """
        q_base: 4x4 homogeneous transform of floating base in world
        v_base: 6-dim spatial twist of floating base in world
        q: array of joint positions (size N)
        Returns:
            T_world: list of 4x4 transforms for each link (0..N)
            v_world: list of 6x1 twists for each link (0..N)
        """
        T_world = [None] * len(self.links)
        v_world = [None] * len(self.links)

        # base
        T_world[0] = q_base
        v_world[0] = v_base.reshape(6, 1)

        for i in range(1, len(self.links)):
            link = self.links[i]
            parent = link.parent
            # joint transform from parent to link
            T_joint = exp_se3(link.S.flatten(), q[i-1])
            T_world[i] = T_world[parent] @ link.T_parent_to_i0 @ T_joint

            # spatial velocity recursion
            X_i_parent = adjoint(link.T_parent_to_i0 @ T_joint)
            v_world[i] = X_i_parent @ v_world[parent] + link.S * 0.0
            # The term link.S * 0.0 is a placeholder for adding S * qdot[i-1]
            # once joint velocities are provided.

        return T_world, v_world

    def link_jacobian(self, q_base, q, link_index):
        """
        Compute whole-body Jacobian for a given link index.
        For simplicity, ignore base orientation here and treat J_base as identity.
        """
        # First compute all world transforms
        T_world, _ = self.forward_kinematics(q_base, np.zeros(6), q)

        # Base part of Jacobian: identity (6x6) in this simplified example
        J_base = np.eye(6)

        # Joint part
        n = self.N
        J_jnt = np.zeros((6, n))

        # Walk from link to base collecting ancestor joints
        i = link_index
        visited = []
        while i != 0:
            visited.append(i)
            i = self.links[i].parent
        visited = visited[::-1]  # from base to link

        for idx, i in enumerate(visited):
            link = self.links[i]
            # column index in q-vector
            j = i - 1
            T_i = T_world[link_index]
            T_j = T_world[i]
            T_i_j = np.linalg.inv(T_j) @ T_i
            X_i_j = adjoint(T_i_j)
            J_jnt[:, j] = (X_i_j @ link.S).flatten()

        J = np.concatenate([J_base, J_jnt], axis=1)
        return J
      
