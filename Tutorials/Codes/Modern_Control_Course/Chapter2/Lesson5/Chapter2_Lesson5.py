import sympy as sp

A = sp.Matrix([[2, 1, 0],
               [0, 2, 0],
               [0, 0, 3]])

# Exact Jordan decomposition: A = P*J*P^{-1}
P, J = A.jordan_form()   # SymPy returns (P, J) with J Jordan, columns of P a Jordan basis

print("J =")
sp.pprint(J)
print("\nCheck A == P*J*P^{-1}:", A == P*J*P.inv())

# Inspect generalized eigenspace dimensions for lambda=2
lam = sp.Integer(2)
d1 = (A - lam*sp.eye(3)).nullspace()
d2 = ((A - lam*sp.eye(3))**2).nullspace()
print("\nnullity k=1:", len(d1), "nullity k=2:", len(d2))

# Demonstrate the power formula for the Jordan block J2(2)
J2 = sp.Matrix([[2, 1],
                [0, 2]])
k = sp.Symbol('k', integer=True, nonnegative=True)
# Closed form for J2^k: [[2^k, k*2^(k-1)], [0, 2^k]]
J2k = sp.Matrix([[2**k, k*2**(k-1)],
                 [0, 2**k]])
print("\nJ2^k formula verified for k=5:", (J2**5) == J2k.subs(k, 5))
