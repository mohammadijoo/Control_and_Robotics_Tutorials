import numpy as np

def gauss_jordan_inverse(A):
    A = np.array(A, dtype=float)
    n = A.shape[0]
    I = np.eye(n)
    Aug = np.hstack([A, I])

    # Forward elimination with partial pivoting
    for col in range(n):
        pivot = np.argmax(np.abs(Aug[col:, col])) + col
        Aug[[col, pivot]] = Aug[[pivot, col]]

        piv = Aug[col, col]
        if np.isclose(piv, 0.0):
            raise ValueError("Matrix is singular or nearly singular.")

        Aug[col, :] = Aug[col, :] / piv

        for row in range(n):
            if row == col:
                continue
            factor = Aug[row, col]
            Aug[row, :] = Aug[row, :] - factor * Aug[col, :]

    return Aug[:, n:]

def similarity_transform(A, T):
    # Prefer solving rather than explicit inverse in larger problems;
    # kept explicit here for pedagogy.
    Tinv = gauss_jordan_inverse(T)
    return Tinv @ A @ T

# Example
A = np.array([[2.0, 1.0],
              [0.0, 3.0]])
T = np.array([[1.0, 1.0],
              [0.0, 1.0]])

A_tilde = similarity_transform(A, T)

# Invariants check (eigenvalues should match)
eigA = np.linalg.eigvals(A)
eigAt = np.linalg.eigvals(A_tilde)

print("A =", A)
print("A_tilde =", A_tilde)
print("eig(A) =", eigA)
print("eig(A_tilde) =", eigAt)
print("trace equal:", np.isclose(np.trace(A), np.trace(A_tilde)))
print("det equal:", np.isclose(np.linalg.det(A), np.linalg.det(A_tilde)))
      
