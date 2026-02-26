import numpy as np
import control as ct

# Mechanical and electrical design (example from Section 5)
J = 0.02       # kg m^2
B = 0.01       # N m s/rad
N = 10.0
Kt = 0.05      # N m/A
Ke = 0.05      # V s/rad
R = 2.0        # ohm

# Controller gains from co-design
Kp = 32.0
Kd = 1.7

# Derived actuator parameters
Beq = B + (N**2) * Kt * Ke / R
Ka = N * Kt / R

# Closed-loop transfer function from q_ref to q
# G(s) = Ka / (J s^2 + Beq s)
# C(s) = Kp + Kd s
num = [Ka * Kp]
den = [J, Beq + Ka * Kd, Ka * Kp]
Gcl = ct.TransferFunction(num, den)

# Step response for a 0.5 rad command
t = np.linspace(0, 1.0, 1000)
t, y = ct.step_response(0.5 * Gcl, T=t)

# Approximate natural frequency and damping
omega_n = np.sqrt(Ka * Kp / J)
zeta = (Beq + Ka * Kd) / (2.0 * omega_n * J)

print("omega_n =", omega_n)
print("zeta =", zeta)

# (Optional) plot using matplotlib
import matplotlib.pyplot as plt
plt.plot(t, y)
plt.xlabel("time [s]")
plt.ylabel("q(t) [rad]")
plt.grid(True)
plt.show()
      
