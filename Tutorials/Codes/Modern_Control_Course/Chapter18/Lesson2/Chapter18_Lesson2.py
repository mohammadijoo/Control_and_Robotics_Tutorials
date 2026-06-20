# Chapter18_Lesson2.py
"""
Conceptual construction of Jordan canonical form for a controlled example.

Libraries:
    sympy: exact symbolic linear algebra
    numpy/scipy/control can be used later for numerical state-space work,
    but exact Jordan construction should not be treated as a floating-point
    design algorithm.
"""

import sympy as sp


def algebraic_multiplicity(A: sp.Matrix, lam: sp.Expr) -> int:
    """Return the algebraic multiplicity of lam in det(sI - A)."""
    s = sp.symbols("s")
    factors = sp.factor_list(A.charpoly(s).as_expr())[1]
    for factor, exponent in factors:
        if sp.simplify(factor.subs(s, lam)) == 0:
            return int(exponent)
    return 0


def nullity(M: sp.Matrix) -> int:
    """Compute dim ker(M) exactly."""
    return M.cols - M.rank()


def nullity_sequence(A: sp.Matrix, lam: sp.Expr, max_power: int) -> list[int]:
    """Return [n_0, n_1, ..., n_max_power], where n_k = dim ker((A-lam I)^k)."""
    N = A - lam * sp.eye(A.rows)
    values = [0]
    for k in range(1, max_power + 1):
        values.append(nullity(N**k))
    return values


def block_sizes_from_nullities(nullities: list[int], algebraic_mult: int) -> list[int]:
    """
    If n_k = dim ker((A-lam I)^k), then
        b_k = n_k - n_{k-1}
    is the number of Jordan blocks of size at least k.
    The number of blocks of size exactly k is b_k - b_{k+1}.
    """
    trimmed = [nullities[0]]
    for value in nullities[1:]:
        trimmed.append(value)
        if value == algebraic_mult:
            break

    b_at_least = [trimmed[k] - trimmed[k - 1] for k in range(1, len(trimmed))]
    b_at_least.append(0)

    sizes = []
    for k in range(1, len(b_at_least)):
        exact_count = b_at_least[k - 1] - b_at_least[k]
        sizes.extend([k] * exact_count)

    return sorted(sizes, reverse=True)


def jordan_block(lam: sp.Expr, size: int) -> sp.Matrix:
    """Create one Jordan block J_size(lam)."""
    B = lam * sp.eye(size)
    for i in range(size - 1):
        B[i, i + 1] = 1
    return B


def build_example() -> sp.Matrix:
    """Create A = P J P^{-1}, so the expected Jordan structure is known."""
    J = sp.diag(jordan_block(2, 3), jordan_block(2, 1), jordan_block(-1, 2))
    P = sp.eye(6)
    for i in range(5):
        P[i, i + 1] = 1
    return sp.simplify(P * J * P.inv())


def main() -> None:
    A = build_example()
    print("A =")
    sp.print_latex(A)
    print(A)

    s = sp.symbols("s")
    print("\nCharacteristic polynomial:")
    print(sp.factor(A.charpoly(s).as_expr()))

    print("\nMinimal-polynomial prediction from block sizes:")
    print("(s - 2)^3 (s + 1)^2")

    for lam in [sp.Integer(2), sp.Integer(-1)]:
        alg = algebraic_multiplicity(A, lam)
        nullities = nullity_sequence(A, lam, A.rows)
        sizes = block_sizes_from_nullities(nullities, alg)

        print(f"\nlambda = {lam}")
        print("algebraic multiplicity =", alg)
        print("nullities n_k =", nullities[: alg + 1])
        print("Jordan block sizes =", sizes)

    Pj, Jj = A.jordan_form()
    print("\nSymPy Jordan form J:")
    print(Jj)
    print("\nVerification P^{-1} A P - J =")
    print(sp.simplify(Pj.inv() * A * Pj - Jj))


if __name__ == "__main__":
    main()
