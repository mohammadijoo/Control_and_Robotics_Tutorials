# ===== Code block 1 extracted from Chapter2/Lesson2.html =====
import numpy as np

def build_matrix_from_images(images):
    """
    images: list of m-vectors [T(e1), T(e2), ..., T(en)]
    returns A in R^{m x n} with columns equal to images
    """
    return np.column_stack(images)

# Example: T: R^3 -> R^2 given by
# T(e1) = [1, 0]^T, T(e2) = [2, -1]^T, T(e3) = [0, 3]^T
Te1 = np.array([1.0, 0.0])
Te2 = np.array([2.0, -1.0])
Te3 = np.array([0.0, 3.0])

A = build_matrix_from_images([Te1, Te2, Te3])

x = np.array([1.0, -2.0, 0.5])
Tx = A @ x

print("A=\n", A)
print("x=", x)
print("T(x)=A x =", Tx)

# Numerical linearity check on random vectors
rng = np.random.default_rng(0)
u = rng.standard_normal(3)
v = rng.standard_normal(3)
alpha = 1.7

left = A @ (u + v)
right = (A @ u) + (A @ v)
print("Additivity error norm:", np.linalg.norm(left - right))

left2 = A @ (alpha * v)
right2 = alpha * (A @ v)
print("Homogeneity error norm:", np.linalg.norm(left2 - right2))

# Composition: S: R^2 -> R^2 with matrix B
B = np.array([[0.0, 1.0],
              [-1.0, 0.0]])  # 90-degree rotation in the plane

# (S o T)(x) = B (A x) = (B A) x
BA = B @ A
print("Composite matrix BA=\n", BA)
print("Composite output:", (BA @ x))
      

# ===== Code block 2 extracted from Chapter2/Lesson2.html =====
import sympy as sp

A = sp.Matrix([[1, 2, 0],
               [0, -1, 3]])
B = sp.Matrix([[0, 1],
               [-1, 0]])

x1, x2, x3 = sp.symbols('x1 x2 x3')
x = sp.Matrix([x1, x2, x3])

expr1 = B*(A*x)
expr2 = (B*A)*x
print("Symbolic equality check:", sp.simplify(expr1 - expr2) == sp.Matrix([0, 0]))
      
