import sympy as sp
import numpy as np

# Symbolic variables
x, y, phi = sp.symbols('x y phi', real=True)
L1, L2, L3 = sp.symbols('L1 L2 L3', positive=True)

# Geometry parameters (example values, change to your mechanism)
B1 = sp.Matrix([0.0, 0.0])
B2 = sp.Matrix([1.0, 0.0])
B3 = sp.Matrix([0.5, sp.sqrt(3) / 2])

P1 = sp.Matrix([0.0, -0.2])
P2 = sp.Matrix([0.2, 0.0])
P3 = sp.Matrix([-0.2, 0.0])

p = sp.Matrix([x, y])
R = sp.Matrix([[sp.cos(phi), -sp.sin(phi)],
               [sp.sin(phi),  sp.cos(phi)]])
S = sp.Matrix([[0, -1],
               [1,  0]])

def leg_vector(B, P):
    return p + R * P - B

d1 = leg_vector(B1, P1)
d2 = leg_vector(B2, P2)
d3 = leg_vector(B3, P3)

Phi1 = d1.dot(d1) - L1**2
Phi2 = d2.dot(d2) - L2**2
Phi3 = d3.dot(d3) - L3**2

Phi = sp.Matrix([Phi1, Phi2, Phi3])
q_vec = sp.Matrix([L1, L2, L3])
x_vec = sp.Matrix([x, y, phi])

Jq_sym = Phi.jacobian(q_vec)
Jx_sym = Phi.jacobian(x_vec)

print("J_q (symbolic):")
sp.pprint(Jq_sym)
print("\nJ_x (symbolic):")
sp.pprint(Jx_sym)

# Lambdify for numerical evaluation
Jq_fun = sp.lambdify((L1, L2, L3, x, y, phi), Jq_sym, 'numpy')
Jx_fun = sp.lambdify((L1, L2, L3, x, y, phi), Jx_sym, 'numpy')

def jacobians_3rpr(L_vals, pose):
    """
    L_vals: array-like [L1, L2, L3]
    pose:   array-like [x, y, phi]
    Returns J_f, J_q, J_x evaluated numerically.
    """
    L1v, L2v, L3v = L_vals
    xv, yv, phiv = pose
    Jq = np.array(Jq_fun(L1v, L2v, L3v, xv, yv, phiv), dtype=float)
    Jx = np.array(Jx_fun(L1v, L2v, L3v, xv, yv, phiv), dtype=float)

    # Solve for J_f = - J_x^{-1} J_q
    Jx_inv = np.linalg.inv(Jx)
    Jf = - Jx_inv @ Jq
    return Jf, Jq, Jx

if __name__ == "__main__":
    L_vals = [0.8, 0.9, 0.85]
    pose = [0.1, 0.2, 0.1]
    Jf, Jq, Jx = jacobians_3rpr(L_vals, pose)
    print("J_f =\n", Jf)
      
