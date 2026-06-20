import numpy as np

# ---------- Geometry of a symmetric 3-RPR robot ----------
# Base anchor points (equilateral triangle of radius Rb)
Rb = 1.0
Ab = np.array([
    [Rb, 0.0],
    [Rb * np.cos(2.0 * np.pi / 3.0), Rb * np.sin(2.0 * np.pi / 3.0)],
    [Rb * np.cos(4.0 * np.pi / 3.0), Rb * np.sin(4.0 * np.pi / 3.0)]
])

# Platform anchor points (equilateral triangle of radius Rp) in platform frame
Rp = 0.5
alpha = np.array([0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0])
b_local = np.stack([Rp * np.cos(alpha), Rp * np.sin(alpha)], axis=1)

def forward_points(x_vec):
    """
    Compute platform points p_i in base frame.
    x_vec = [x, y, phi].
    """
    x, y, phi = x_vec
    R = np.array([[np.cos(phi), -np.sin(phi)],
                  [np.sin(phi),  np.cos(phi)]])
    p = np.empty_like(b_local)
    for i in range(3):
        p[i, :] = np.array([x, y]) + R @ b_local[i, :]
    return p

def compute_AB(x_vec, rho):
    """
    Build A and B matrices for the 3-RPR mechanism.
    A: 3x3, B: 3x3 diagonal.
    """
    p = forward_points(x_vec)
    A = np.zeros((3, 3))
    B = np.zeros((3, 3))

    x, y, phi = x_vec
    for i in range(3):
        # leg vector and its norm
        d = p[i, :] - Ab[i, :]
        # partial of p_i w.r.t. x, y, phi
        ri = Rp
        ang = phi + alpha[i]
        dp_dxi = np.array([[1.0, 0.0, -ri * np.sin(ang)],
                           [0.0, 1.0,  ri * np.cos(ang)]])
        # A row: 2 (p_i - A_i)^T * dp_dxi
        A[i, :] = 2.0 * d @ dp_dxi

        # B diagonal: dPhi_i/drho_i = -2 rho_i
        B[i, i] = -2.0 * rho[i]

    return A, B

def classify_singularity(A, B, tol=1e-6):
    """
    Classify configuration based on A and B.
    """
    # determinants (may be near zero)
    detA = np.linalg.det(A)
    detB = np.linalg.det(B)

    # singular values for rank and condition number
    sA = np.linalg.svd(A, compute_uv=False)
    sB = np.linalg.svd(B, compute_uv=False)

    rankA = np.sum(sA > tol)
    rankB = np.sum(sB > tol)

    kappaA = (sA[0] / sA[-1]) if sA[-1] > tol else np.inf
    kappaB = (sB[0] / sB[-1]) if sB[-1] > tol else np.inf

    m = A.shape[0]
    type_I = rankB < m
    type_II = rankA < m

    if type_I and type_II:
        singular_type = "Type III (combined)"
    elif type_I:
        singular_type = "Type I (serial)"
    elif type_II:
        singular_type = "Type II (parallel)"
    else:
        singular_type = "Regular"

    return {
        "type": singular_type,
        "detA": detA,
        "detB": detB,
        "rankA": int(rankA),
        "rankB": int(rankB),
        "kappaA": float(kappaA),
        "kappaB": float(kappaB),
    }

# ---------- Example usage ----------
if __name__ == "__main__":
    # Example configuration: center pose with moderate orientation
    x_vec = np.array([0.0, 0.0, 0.3])
    rho = np.array([1.0, 1.0, 1.0])

    A, B = compute_AB(x_vec, rho)
    info = classify_singularity(A, B)

    print("A =\n", A)
    print("B =\n", B)
    print("Classification:", info["type"])
    print("det(A) =", info["detA"], "det(B) =", info["detB"])
    print("rank(A) =", info["rankA"], "rank(B) =", info["rankB"])
    print("kappa(A) =", info["kappaA"], "kappa(B) =", info["kappaB"])
      
