import numpy as np

class DeformableRopeState:
    """
    Rope modeled as n point masses in R^3.
    State s = [q; qdot] with q, qdot in R^{3n}.
    """
    def __init__(self, n_nodes: int):
        self.n = n_nodes
        # Positions and velocities as arrays of shape (n, 3)
        self.positions = np.zeros((n_nodes, 3), dtype=float)
        self.velocities = np.zeros((n_nodes, 3), dtype=float)

    @property
    def q(self) -> np.ndarray:
        """Flattened position vector in R^{3n}."""
        return self.positions.reshape(-1)

    @property
    def qdot(self) -> np.ndarray:
        """Flattened velocity vector in R^{3n}."""
        return self.velocities.reshape(-1)

    def state_vector(self) -> np.ndarray:
        """Return s = [q; qdot] in R^{6n}."""
        return np.concatenate([self.q, self.qdot], axis=0)

    def from_state_vector(self, s: np.ndarray):
        """Load positions and velocities from s in R^{6n}."""
        assert s.shape[0] == 6 * self.n
        q = s[:3*self.n]
        qdot = s[3*self.n:]
        self.positions = q.reshape(self.n, 3)
        self.velocities = qdot.reshape(self.n, 3)

def build_chain_adjacency(n_nodes: int):
    """
    Build edge list for a chain graph 1--2--...--n.
    Returns list of (i, j) pairs with 0-based indices.
    """
    edges = [(i, i + 1) for i in range(n_nodes - 1)]
    return edges

def stiffness_matrix_from_edges(n_nodes: int, edges, k_spring: float) -> np.ndarray:
    """
    Construct a simple scalar stiffness matrix for a 1D chain,
    then lift it to 3D by Kronecker product with I_3.
    """
    L = np.zeros((n_nodes, n_nodes), dtype=float)
    for i, j in edges:
        L[i, i] += 1.0
        L[j, j] += 1.0
        L[i, j] -= 1.0
        L[j, i] -= 1.0
    # Scalar Laplacian scaled by k_spring
    K_scalar = k_spring * L
    # Lift to 3D: K = kron(K_scalar, I_3)
    K = np.kron(K_scalar, np.eye(3))
    return K

# Example usage
n = 10
rope = DeformableRopeState(n)
edges = build_chain_adjacency(n)
K = stiffness_matrix_from_edges(n, edges, k_spring=10.0)

print("State dimension:", rope.state_vector().shape[0])  # should be 6n

# Modal basis via eigen-decomposition of K (assuming unit masses)
eigvals, eigvecs = np.linalg.eigh(K)
# Keep r lowest non-zero modes (ignore the rigid-motion modes)
r = 4
Phi = eigvecs[:, 3:3+r]  # skip 3 smallest eigenvalues (approx rigid modes)

def project_to_modal(q: np.ndarray, q0: np.ndarray, Phi: np.ndarray) -> np.ndarray:
    """
    Compute modal coordinates z given full configuration q and rest q0.
    Phi has shape (3n, r).
    """
    return Phi.T @ (q - q0)

def reconstruct_from_modal(z: np.ndarray, q0: np.ndarray, Phi: np.ndarray) -> np.ndarray:
    """
    Reconstruct configuration from modal coordinates.
    """
    return q0 + Phi @ z
      
