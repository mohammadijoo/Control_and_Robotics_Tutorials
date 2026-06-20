# Chapter18_Lesson1.py
# Jordan blocks and generalized eigenvectors for a state-space matrix.
# Required libraries:
#   sympy  : exact symbolic linear algebra
#   numpy  : numerical arrays if students want to extend the example
#
# Install:
#   pip install sympy numpy

import sympy as sp


def nullity(M: sp.Matrix) -> int:
    """Return dim Ker(M)."""
    return M.shape[1] - M.rank()


def print_vector(name: str, v: sp.Matrix) -> None:
    print(f"{name} =")
    sp.print_latex(v)
    print(v)


def main() -> None:
    # A contains one Jordan block of size 3 at lambda=2 and one scalar block at lambda=-1.
    A = sp.Matrix([
        [2, 1, 0, 0],
        [0, 2, 1, 0],
        [0, 0, 2, 0],
        [0, 0, 0, -1],
    ])

    lam = sp.Integer(2)
    n = A.rows
    I = sp.eye(n)
    N = A - lam * I

    print("A =")
    sp.pprint(A)
    print("\nN = A - lambda I =")
    sp.pprint(N)

    print("\nGrowth of generalized eigenspaces Ker((A-lambda I)^k):")
    for k in range(1, 5):
        Nk = N**k
        print(f"k={k}: rank={Nk.rank()}, nullity={nullity(Nk)}")

    # A Jordan chain of length 3 for lambda=2:
    # (A-lambda I)v1 = 0
    # (A-lambda I)v2 = v1
    # (A-lambda I)v3 = v2
    v3 = sp.Matrix([0, 0, 1, 0])
    v2 = N * v3
    v1 = N * v2

    print("\nJordan chain for lambda=2:")
    print_vector("v1", v1)
    print_vector("v2", v2)
    print_vector("v3", v3)

    print("\nChain verification:")
    print("N*v1 =", N * v1)
    print("N*v2 =", N * v2)
    print("N*v3 =", N * v3)

    # Add eigenvector for lambda=-1 to obtain a complete basis.
    w = sp.Matrix([0, 0, 0, 1])
    T = sp.Matrix.hstack(v1, v2, v3, w)
    J = T.inv() * A * T

    print("\nTransformation matrix T = [v1 v2 v3 w]:")
    sp.pprint(T)
    print("\nJ = T^{-1} A T:")
    sp.pprint(J)

    # Exponential of the 3x3 Jordan block J3 = lambda I + S, where S^3=0.
    t = sp.symbols("t", real=True)
    S = sp.Matrix([
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0],
    ])
    exp_J3 = sp.exp(lam * t) * (sp.eye(3) + t * S + (t**2 / 2) * (S**2))

    print("\nexp(J3*t) for the size-3 Jordan block:")
    sp.pprint(exp_J3)


if __name__ == "__main__":
    main()
