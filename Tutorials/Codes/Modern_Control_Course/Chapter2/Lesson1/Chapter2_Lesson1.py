import numpy as np

def rref(A, tol=1e-10):
    """
    Reduced Row Echelon Form via Gauss-Jordan elimination.
    Returns (R, pivot_cols).
    A is copied to float.
    """
    R = A.astype(float).copy()
    m, n = R.shape
    pivot_cols = []
    row = 0

    for col in range(n):
        if row >= m:
            break

        # Find pivot row with max abs value in current column (partial pivoting)
        pivot = np.argmax(np.abs(R[row:, col])) + row
        if abs(R[pivot, col]) < tol:
            continue

        # Swap pivot row into position
        if pivot != row:
            R[[row, pivot]] = R[[pivot, row]]

        # Normalize pivot row
        R[row] = R[row] / R[row, col]

        # Eliminate other entries in this column
        for r in range(m):
            if r != row:
                R[r] = R[r] - R[r, col] * R[row]

        pivot_cols.append(col)
        row += 1

    # Clean tiny numerical noise
    R[np.abs(R) < tol] = 0.0
    return R, pivot_cols

def basis_from_columns(V, tol=1e-10):
    """
    V: matrix whose columns are candidate vectors.
    Returns (basis_cols, pivot_cols, R).
    """
    R, piv = rref(V, tol=tol)
    basis_cols = V[:, piv] if len(piv) > 0 else V[:, :0]
    return basis_cols, piv, R

# Example: 4D vectors v1..v4 (columns)
V = np.array([
    [1, 2, 0, 1],
    [0, 1, 1, 1],
    [1, 3, 1, 2],
    [0, 0, 1, 1],
], dtype=float)

B, piv, R = basis_from_columns(V)
print("Pivot columns:", piv)
print("Dimension of span:", len(piv))
print("Basis vectors (as columns):\n", B)
print("RREF:\n", R)

# Coordinate representation in the extracted basis:
# Given a target vector v, solve B*c = v if v is in span(B).
v = np.array([3, 2, 5, 2], dtype=float)
if B.shape[1] > 0:
    # Solve least squares; exact if v in span(B)
    c, residuals, _, _ = np.linalg.lstsq(B, v, rcond=None)
    print("Coordinates c (least-squares):", c)
    print("Reconstruction B*c:", B @ c)
    print("Residual norm:", np.linalg.norm(B @ c - v))
      
