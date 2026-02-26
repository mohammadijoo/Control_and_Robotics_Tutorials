import sympy as sp

# Symbolic variables
s, t = sp.symbols('s t', real=True, positive=True)

# Example: Y(s) = 1 / (s*(s + 3))
Y = 1 / (s * (s + 3))

# Partial fraction expansion (apart)
Y_part = sp.apart(Y, s)
print("Partial fraction expansion:", Y_part)

# Inverse Laplace transform
y_t = sp.inverse_laplace_transform(Y, s, t)
print("y(t) =", sp.simplify(y_t))

# Using python-control for a similar rational function
# pip install control
import control

# Transfer function with numerator 1 and denominator s*(s+3)
num = [1]
den = [1, 3, 0]  # s^2 + 3s + 0 = s*(s+3)
G = control.TransferFunction(num, den)

print("G(s) =", G)

# Step response (useful in servo/robot joint models)
t_vec, y_step = control.step_response(G)

# t_vec and y_step can be plotted or compared with the analytic y(t)
