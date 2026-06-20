import numpy as np
import matplotlib.pyplot as plt

# Parameters (can correspond to a simple filtered sensor signal on a robot)
A = 1.0          # input amplitude
phi_u = np.deg2rad(30.0)  # input phase [rad]
omega = 10.0     # angular frequency [rad/s]
tau = 0.05       # time constant [s]

# Time vector
t = np.linspace(0.0, 0.5, 2000)

# 1) Build input phasor and time signal
U_phasor = A * np.exp(1j * phi_u)         # U* = A e^{j phi}
u_t = np.real(U_phasor * np.exp(1j * omega * t))

# 2) First-order complex gain H(j omega)
H_jw = 1.0 / (1.0 + 1j * tau * omega)     # H(j omega) = 1 / (1 + j tau omega)
magH = np.abs(H_jw)
phaseH = np.angle(H_jw)

# 3) Output phasor and time signal
Y_phasor = H_jw * U_phasor
y_t = np.real(Y_phasor * np.exp(1j * omega * t))

print("H(j omega) magnitude:", magH)
print("H(j omega) phase [deg]:", np.rad2deg(phaseH))

# Plot input and output
plt.figure()
plt.plot(t, u_t, label="input u(t)")
plt.plot(t, y_t, label="output y(t)")
plt.xlabel("t [s]")
plt.ylabel("signal")
plt.legend()
plt.title("Sinusoidal steady-state via phasors")
plt.grid(True)
plt.show()

# Robotics/control note:
# The 'python-control' library can model robot joint actuators and evaluate their
# frequency response. Phasor-based reasoning underlies Bode plots and resonance
# analysis used in robotic servo-loop tuning.
