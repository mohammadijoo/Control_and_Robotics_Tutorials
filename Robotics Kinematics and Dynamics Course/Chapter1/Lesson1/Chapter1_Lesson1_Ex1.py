def rref(A, tol=1e-12):
    """
    Compute a reduced row echelon form of A (copy), returning R and pivot column indices.
    No pivoting (for simplicity); not numerically robust but fine for small examples.
    """
    A = A.astype(float).copy()
    m, n = A.shape
    pivots = []
    row = 0
    for col in range(n):
        if row >= m:
            break
        # Find a row with nonzero entry in this column
        pivot_row = None
        for r in range(row, m):
            if abs(A[r, col]) > tol:
                pivot_row = r
                break
        if pivot_row is None:
            continue  # no pivot in this column
        # Swap into position
        if pivot_row != row:
            A[[row, pivot_row], :] = A[[pivot_row, row], :]
        # Normalize pivot row
        A[row, :] /= A[row, col]
        # Eliminate other entries in this column
        for r in range(m):
            if r != row:
                A[r, :] -= A[r, col] * A[row, :]
        pivots.append(col)
        row += 1
    return A, pivots

A = np.array([[1.0, 1.0, 0.0],
              [0.0, 1.0, 1.0]])
R, pivots = rref(A)
print("RREF(A) =\n", R)
print("Pivot columns =", pivots)
rank = len(pivots)

# Construct a null-space basis by setting free variables to parameters
m, n = A.shape
free_cols = [j for j in range(n) if j not in pivots]
basis = []
for free in free_cols:
    z = np.zeros(n)
    z[free] = 1.0
    # Solve for pivot variables from RREF: pivot columns are leading 1's
    for i, col in enumerate(pivots):
        z[col] = -R[i, free]
    basis.append(z)

N_manual = np.column_stack(basis) if basis else np.zeros((n, 0))
print("Manual null-space basis:\n", N_manual)
      
