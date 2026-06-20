import numpy as np
import control  # pip install control
import matplotlib.pyplot as plt

# Physical parameters (example: velocity actuator in a robot joint)
K = 5.0      # static gain
tau = 0.1    # time constant [s]

# Transfer function G(s) = K / (tau s + 1)
num = [K]
den = [tau, 1.0]
G = control.tf(num, den)

# Sinusoidal input parameters
U = 1.0          # input amplitude
omega = 10.0     # rad/s
t_final = 2.0
n_points = 2000
t = np.linspace(0.0, t_final, n_points)
u = U * np.sin(omega * t)

# Simulate time response to sinusoidal input
t_out, y_out, x_out = control.forced_response(G, T=t, U=u)

# Theoretical steady-state amplitude and phase
Gjw = control.evalfr(G, 1j * omega)
mag = abs(Gjw)
phase = np.angle(Gjw)  # radians

print("Complex gain G(jw) =", Gjw)
print("Amplitude ratio |G(jw)| =", mag)
print("Phase shift arg(G(jw)) [rad] =", phase)

# Optional plotting (remove or adapt for headless servers)
plt.figure()
plt.plot(t_out, u, label="input u(t)")
plt.plot(t_out, y_out, label="output y(t)")
plt.xlabel("t [s]")
plt.legend()
plt.grid(True)
plt.show()
