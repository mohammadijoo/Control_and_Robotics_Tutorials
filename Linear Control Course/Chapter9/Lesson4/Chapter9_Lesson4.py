import numpy as np
import sympy as sp

# Example: p(s,K) = s^3 + 6 s^2 + 8 s + K
s, K = sp.symbols("s K", real=True)
p = s**3 + 6*s**2 + 8*s + K

def routh_cubic_symbolic(poly, K_symbol):
    """
    Compute the first column of the Routh table for a cubic
    poly(s,K) = s^3 + a1 s^2 + a2 s + a3(K).
    """
    # Extract coefficients in descending powers of s
    a3, a2, a1, a0 = sp.Poly(poly, s).all_coeffs()
    # a3 should be 1 for this example
    row_s3 = [a3, a1]
    row_s2 = [a2, a0]
    b1 = (row_s2[0]*row_s3[1] - row_s3[0]*row_s2[1]) / row_s2[0]
    row_s1 = [sp.simplify(b1), 0]
    row_s0 = [row_s2[1], 0]
    return row_s3, row_s2, row_s1, row_s0

row_s3, row_s2, row_s1, row_s0 = routh_cubic_symbolic(p, K)
print("Routh rows (symbolic):")
print("s^3:", row_s3)
print("s^2:", row_s2)
print("s^1:", sp.simplify(row_s1[0]), 0)
print("s^0:", row_s0[0], 0)

# Solve for K where s^1 row becomes zero
Kcrit = sp.solve(sp.Eq(row_s1[0], 0), K)
print("Critical K (Routh zero row):", Kcrit)

# Auxiliary polynomial at Kcrit
Kcrit_val = Kcrit[0]
A = row_s2[0]*s**2 + row_s2[1].subs(K, Kcrit_val)
print("Auxiliary polynomial:", sp.factor(A))

# Evaluate imaginary-axis roots of A(s)
omega_sq = -sp.factor(A.subs(s, sp.I*sp.symbols("omega"))).as_real_imag()[0] / \
           sp.symbols("omega")**2
print("Crossing frequency squared (from A):", sp.simplify(omega_sq))

# Numerical confirmation with python-control (if installed)
try:
    import control as ctl
    # Define open-loop transfer function G(s) = K / (s (s+2) (s+4)), K will be applied later
    G = ctl.tf([1], [1, 6, 8, 0])

    def max_real_pole(Kval):
        T = ctl.feedback(Kval*G, 1)  # closed-loop transfer function
        poles = ctl.pole(T)
        return np.max(np.real(poles))

    K_vals = np.linspace(0, 80, 161)
    sign_changes = []
    previous = max_real_pole(K_vals[0])
    for Kv in K_vals[1:]:
        current = max_real_pole(Kv)
        if previous < 0 and current > 0:
            sign_changes.append(Kv)
        previous = current

    print("Approximate crossing K from numerical scan:", sign_changes[:3])
except ImportError:
    print("python-control not installed; skipping numeric confirmation.")
