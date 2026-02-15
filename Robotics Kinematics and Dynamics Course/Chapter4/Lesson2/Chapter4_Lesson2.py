import numpy as np

class SerialChain:
    def __init__(self, n_links):
        # links are 0,...,n_links-1, base is 0, end-effector is n_links-1
        self.n_links = n_links
        # parent[i] = index of parent of link i; parent[0] = -1 (no parent)
        self.parent = [-1] + [i for i in range(n_links - 1)]
        # each joint transform is a function q_i -> 4x4 homogeneous matrix
        self.joint_tf = [None] * n_links

    def set_joint_transform(self, i, f_q_to_T):
        """
        i: link index > 0
        f_q_to_T: callable q -> 4x4 np.array in SE(3)
        """
        assert 0 <= i < self.n_links
        self.joint_tf[i] = f_q_to_T

    def forward_transforms(self, q):
        """
        q: array-like of length n_links-1 (q[0] for joint 1, etc.).
        Returns list T[i] = ^0 T_i(q).
        """
        T = [np.eye(4) for _ in range(self.n_links)]
        for i in range(1, self.n_links):
            p = self.parent[i]
            T_pi = self.joint_tf[i](q[i - 1])  # ^{p(i)} T_i(q_i)
            T[i] = T[p] @ T_pi
        return T

# Example: 3-link planar RRR chain with unit-length links in the x-y plane
def planar_R_joint(theta, a):
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s; T[1, 0] = s; T[1, 1] = c
    T[0, 3] = a  # translation along x of parent frame
    return T

chain = SerialChain(n_links=4)  # links 0..3, joints 1..3
link_lengths = [1.0, 1.0, 1.0]
for i in range(1, 4):
    a_i = link_lengths[i - 1]
    chain.set_joint_transform(i, lambda th, a=a_i: planar_R_joint(th, a))

q = np.array([0.2, 0.4, -0.3])
T_list = chain.forward_transforms(q)
T_ee = T_list[-1]
print("End-effector position:", T_ee[:3, 3])
      
