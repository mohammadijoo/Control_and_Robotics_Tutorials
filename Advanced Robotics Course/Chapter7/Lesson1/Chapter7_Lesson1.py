import numpy as np

def friction_cone_generators_2d(mu, m_edges):
    """
    Approximate a 2D Coulomb friction cone in the contact frame
    (tangent axis x, normal axis y) by m_edges rays.

    Returns an array Gc of shape (2, m_edges) whose columns are
    unit vectors on or inside the true cone.
    """
    if mu <= 0.0:
        raise ValueError("mu must be positive")

    # True cone half-angle
    alpha = np.arctan(mu)

    # Sample directions between -alpha and +alpha
    angles = np.linspace(-alpha, alpha, m_edges)
    # In contact frame: normal is along +y
    Gc = np.vstack([np.sin(angles), np.cos(angles)])
    return Gc  # shape (2, m_edges)

def planar_contact_wrench_generators(p_obj, mu, m_edges):
    """
    p_obj: 2D contact position in object frame (x, y).
    mu: friction coefficient.
    m_edges: number of edges in the polyhedral cone.

    Returns W (3 x m_edges): wrench generators in the object frame (2D).
    """
    Gc = friction_cone_generators_2d(mu, m_edges)  # (2, m_edges)
    # Forces in object frame assumed identical to contact frame for planar case
    fx = Gc[0, :]
    fy = Gc[1, :]

    # Torque about origin tau_z = p x f (in 2D)
    tau_z = p_obj[0] * fy - p_obj[1] * fx

    W = np.vstack([fx, fy, tau_z])
    return W  # (3, m_edges)

def assemble_planar_grasp_map(contacts, mu, m_edges):
    """
    contacts: list of 2D positions (x, y) for each contact in object frame.
    Returns G (3 x m) where m = len(contacts) * m_edges.
    """
    W_list = []
    for p in contacts:
        W_i = planar_contact_wrench_generators(np.array(p), mu, m_edges)
        W_list.append(W_i)
    return np.hstack(W_list)

# Example: two-finger pinch grasp of a 2D object
mu = 0.8
m_edges = 8
contacts = [(-0.05, 0.0), (0.05, 0.0)]  # left and right

G = assemble_planar_grasp_map(contacts, mu, m_edges)
print("Grasp map shape:", G.shape)

# Check if we can balance gravity wrench (assuming object COM at origin)
mass = 0.5
g = 9.81
w_ext = np.array([0.0, -mass * g, 0.0])  # gravity in -y, no external torque

# Feasibility check via nonnegative combination of generators:
# G * lambda + w_ext = 0, lambda >= 0
# Here we solve a small NNLS problem (or LP). For illustration, try least-squares
# and check if lambda is approximately nonnegative.

lambda_ls, *_ = np.linalg.lstsq(G, -w_ext, rcond=None)
print("Candidate lambda (LS):", lambda_ls)

print("All lambda >= 0 ?", np.all(lambda_ls >= -1e-6))
      
