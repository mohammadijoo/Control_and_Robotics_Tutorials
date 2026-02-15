import sympy as sp

# Symbolic Laplace variable
s = sp.symbols('s')

# Example plant and sensor dynamics (e.g. simple robotic actuator plus encoder)
# G(s) = 10 / (s (s + 2)),   H(s) = 0.5
G = 10 / (s * (s + 2))
H = sp.Rational(1, 2)

# Mason's formula for single forward path and single loop:
# P1 = G, L1 = -G*H, Delta = 1 - L1 = 1 + G*H, Delta1 = 1
T_mason = sp.simplify(G / (1 + G*H))
print("Closed-loop TF from Mason:", T_mason)

# Numerical check using python-control (commonly used in robotics)
import control as ctrl

# Plant and sensor transfer functions in python-control
G_tf = ctrl.TransferFunction([10], [1, 2, 0])   # 10 / (s^2 + 2 s)
H_tf = ctrl.TransferFunction([0.5], [1])        # 0.5

# Negative feedback (unity reference, sensor H)
T_tf = ctrl.feedback(G_tf, H_tf, sign=-1)
print("Closed-loop TF from python-control:", T_tf)

# The rational form of T_tf should match the symbolic T_mason after simplification.
