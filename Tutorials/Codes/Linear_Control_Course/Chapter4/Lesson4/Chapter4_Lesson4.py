import numpy as np
import control as ct  # python-control library

# Example: simple robotic joint actuator model (linearized)
# Plant: Gp(s) = K / (J s^2 + b s)
J = 0.01   # effective inertia
b = 0.1    # viscous friction
K = 1.0    # motor torque constant

s = ct.TransferFunction.s
Gp = K / (J * s**2 + b * s)

# Controller: proportional gain Kc
Kc = 20.0
Gc = Kc

# Sensor: encoder with gain Ks
Ks = 1.0
H = Ks

# Forward path is series of controller and plant
G_forward = ct.series(Gc, Gp)

# Closed-loop transfer function from reference to output (negative feedback)
T = ct.feedback(G_forward, H)  # G_forward / (1 + G_forward*H)

print("Closed-loop transfer function T(s) = Y(s)/R(s):")
print(T)

# --- Low-level implementation using polynomials ---

def convolve(a, b):
    """Convolution of two 1D coefficient arrays."""
    return np.convolve(a, b)

def series(num1, den1, num2, den2):
    num = convolve(num1, num2)
    den = convolve(den1, den2)
    return num, den

def parallel(num1, den1, num2, den2):
    den = convolve(den1, den2)
    num1_ext = convolve(num1, den2)
    num2_ext = convolve(num2, den1)
    num = num1_ext + num2_ext
    return num, den

def feedback(numG, denG, numH=None, denH=None):
    """
    Negative feedback: G / (1 + G H)
    If H is None, assume unity feedback H(s) = 1.
    """
    if numH is None or denH is None:
        numH = np.array([1.0])
        denH = np.array([1.0])

    num = convolve(numG, denH)
    den1 = convolve(denG, denH)
    den2 = convolve(numG, numH)
    # Pad and add denominators
    if den1.size < den2.size:
        den1 = np.pad(den1, (den2.size - den1.size, 0))
    elif den2.size < den1.size:
        den2 = np.pad(den2, (den1.size - den2.size, 0))
    den = den1 + den2
    return num, den

# Example: numerically define Gp(s) and unity feedback
num_Gp = np.array([K])
den_Gp = np.array([J, b, 0.0])  # J s^2 + b s

# Controller is a gain Kc
num_Gc = np.array([Kc])
den_Gc = np.array([1.0])

# Series: controller * plant
num_forward, den_forward = series(num_Gc, den_Gc, num_Gp, den_Gp)

# Closed-loop with unity sensor (H(s) = 1)
num_cl, den_cl = feedback(num_forward, den_forward)

print("Closed-loop numerator coefficients:", num_cl)
print("Closed-loop denominator coefficients:", den_cl)
