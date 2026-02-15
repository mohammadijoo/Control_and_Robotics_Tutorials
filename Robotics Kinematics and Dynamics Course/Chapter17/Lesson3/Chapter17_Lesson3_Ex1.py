import numpy as np

def assemble_centroidal_matrix(I_list, J_list):
    """
    I_list: list of 6x6 spatial inertia matrices I_i^G
    J_list: list of 6 x nq Jacobians J_i^G(q)
    returns A(q): 6 x nq centroidal momentum matrix
    """
    assert len(I_list) == len(J_list)
    nq = J_list[0].shape[1]
    A = np.zeros((6, nq))
    for I_i, J_i in zip(I_list, J_list):
        A += I_i @ J_i
    return A

def centroidal_momentum(I_list, J_list, qdot):
    A = assemble_centroidal_matrix(I_list, J_list)
    return A @ qdot

# Example with dummy data
N = 3
nq = 8
I_list = [np.eye(6) for _ in range(N)]
J_list = [np.random.randn(6, nq) for _ in range(N)]
qdot = np.random.randn(nq)

hG = centroidal_momentum(I_list, J_list, qdot)
print("h_G:", hG)
      
