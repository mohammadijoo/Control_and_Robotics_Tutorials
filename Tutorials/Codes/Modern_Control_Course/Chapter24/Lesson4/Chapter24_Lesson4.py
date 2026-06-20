# Chapter24_Lesson4.py
# Partial Pole Placement and Restricted Eigenstructure
# Dependencies: numpy, scipy
# Install: pip install numpy scipy

import numpy as np
from scipy.linalg import eig, null_space

np.set_printoptions(precision=6, suppress=True)


def partial_pole_placement(A, B, assigned_lambdas, input_directions, keep_basis):
    """Construct K for u = -Kx.

    Assigned eigenpairs satisfy
        (A - lambda_i I) v_i = B g_i,
        K v_i = g_i.

    Kept open-loop modes satisfy
        K n_j = 0.

    Therefore, with X = [V_assigned, N_keep] and Y = [G_assigned, 0],
        K X = Y.
    If X is square nonsingular, K = Y X^{-1}; otherwise use a least-squares
    right solve.
    """
    n = A.shape[0]
    V_cols = []
    G_cols = []

    for lam, g in zip(assigned_lambdas, input_directions):
        v = np.linalg.solve(A - lam * np.eye(n), B @ g)
        V_cols.append(v)
        G_cols.append(g)

    V = np.column_stack(V_cols)
    G = np.column_stack(G_cols)
    X = np.column_stack((V, keep_basis))
    Y = np.column_stack((G, np.zeros((B.shape[1], keep_basis.shape[1]))))

    if X.shape[0] == X.shape[1] and abs(np.linalg.det(X)) > 1e-12:
        K = Y @ np.linalg.inv(X)
    else:
        K = Y @ np.linalg.pinv(X)

    return K, V, G, X, Y


def restricted_mode(A, B, lam, H):
    """Find one mode satisfying H v = 0 and (A-lambda I)v = B g.

    Since v = (A-lambda I)^{-1} B g, the restriction becomes
        H (A-lambda I)^{-1} B g = 0.
    """
    n = A.shape[0]
    M = np.linalg.solve(A - lam * np.eye(n), B)
    S = H @ M
    Gnull = null_space(S)
    if Gnull.size == 0:
        raise ValueError("No nonzero input direction satisfies the restriction.")
    g = Gnull[:, 0]
    v = M @ g
    return v, g, S


# Example system: four states, two independent actuators.
A = np.diag([0.2, 0.6, -0.5, -1.0])
B = np.array([
    [1.0, 0.0],
    [0.3, 1.0],
    [0.2, 0.4],
    [0.1, 0.2]
])

# Assign two unstable/slow modes and keep two stable modes unchanged.
assigned_lambdas = [-2.0, -3.0]
input_directions = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
keep_basis = np.eye(4)[:, 2:4]

K, V, G, X, Y = partial_pole_placement(
    A, B, assigned_lambdas, input_directions, keep_basis
)
Acl = A - B @ K

print("K =")
print(K)
print("Closed-loop eigenvalues =", np.sort_complex(eig(Acl, left=False, right=False)))
print("Condition number of X =", np.linalg.cond(X))
print("Assigned-mode residual ||Acl V - V Lambda|| =",
      np.linalg.norm(Acl @ V - V @ np.diag(assigned_lambdas)))
print("Preservation residual ||K N_keep|| =", np.linalg.norm(K @ keep_basis))

# Restricted eigenstructure example: force the assigned eigenvector to satisfy x3 = x4.
H = np.array([[0.0, 0.0, 1.0, -1.0]])
v_restricted, g_restricted, S = restricted_mode(A, B, -2.0, H)
print("\nRestriction matrix S = H (A-lambda I)^(-1) B =")
print(S)
print("Restricted input direction g =", g_restricted)
print("Restricted eigenvector v =", v_restricted)
print("H v =", H @ v_restricted)
