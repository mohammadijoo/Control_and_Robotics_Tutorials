import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Example parameters for a robot joint actuator
K = 10.0
z = 10.0       # real zero (rad/s)
wn = 20.0      # natural frequency of complex poles
zeta = 0.3     # damping ratio
Tdelay = 0.02  # pure time delay (s)

# Define s as the Laplace variable
s = ct.TransferFunction.s

# Second-order complex pole pair in denominator
G_poles = 1.0 / (1 + 2*zeta * (s/wn) + (s/wn)**2)

# Real zero in numerator
G_zero = 1 + s/z

# Use a first-order Pade approximation for the delay
num_d, den_d = ct.pade(Tdelay, 1)  # (1,1) Pade approximation
G_delay = ct.TransferFunction(num_d, den_d)

# Complete plant model
G = K * G_zero * G_poles * G_delay

# Frequency range relevant for robot joint bandwidth
omega = np.logspace(0, 3, 500)  # 1 to 1000 rad/s

mag, phase, w = ct.bode(G, omega, Plot=False)

# Plot Bode magnitude
plt.figure()
plt.semilogx(w, 20*np.log10(mag))
plt.xlabel("omega (rad/s)")
plt.ylabel("Magnitude (dB)")
plt.title("Bode Magnitude: Robot Joint Plant with Zero, Complex Poles, Delay")
plt.grid(True, which="both")

# Plot Bode phase
plt.figure()
plt.semilogx(w, phase * 180.0/np.pi)
plt.xlabel("omega (rad/s)")
plt.ylabel("Phase (deg)")
plt.title("Bode Phase: Robot Joint Plant with Zero, Complex Poles, Delay")
plt.grid(True, which="both")

plt.show()

# Robotics context (conceptual):
# In a robotics stack, this G(s) may represent a linearized joint model derived from
# robot dynamics (e.g., via roboticstoolbox) around a nominal pose, and the Bode plot
# is used to design joint-space feedback gains.
