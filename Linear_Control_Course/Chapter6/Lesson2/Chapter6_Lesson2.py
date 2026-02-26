import numpy as np
from control import TransferFunction, step_response, pole

def second_order_from_poles(p1, p2):
    """
    Given two poles p1, p2 (complex or real) of a second-order system,
    return (zeta, omega_n).
    """
    # For a second-order system, p(s) = (s - p1)(s - p2)
    #   = s**2 - (p1 + p2)s + p1*p2
    # and p(s) = s**2 + 2 zeta omega_n s + omega_n**2
    s1 = p1
    s2 = p2
    # Sum and product
    s_sum = s1 + s2
    s_prod = s1 * s2

    omega_n = np.sqrt(np.real(s_prod))
    # For complex conjugate or negative real poles, real part is negative
    zeta = -np.real(s_sum) / (2.0 * omega_n)
    return float(zeta), float(omega_n)

def transfer_function_from_zn(zeta, omega_n, k=1.0):
    """
    Construct G(s) = k * omega_n^2 / (s^2 + 2 zeta omega_n s + omega_n^2).
    """
    num = [k * omega_n**2]
    den = [1.0, 2.0 * zeta * omega_n, omega_n**2]
    return TransferFunction(num, den)

# Example: design a joint servo model with zeta = 0.7, omega_n = 8 rad/s
zeta_des = 0.7
omega_n_des = 8.0
G = transfer_function_from_zn(zeta_des, omega_n_des)

print("G(s) =", G)
print("Poles:", pole(G))

# Recover zeta and omega_n from the actual poles:
p = pole(G)
zeta_hat, omega_n_hat = second_order_from_poles(p[0], p[1])
print("Recovered zeta =", zeta_hat, ", omega_n =", omega_n_hat)

# Time response (will be used in later lessons to define performance metrics)
t, y = step_response(G)
