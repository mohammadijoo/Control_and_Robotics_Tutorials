import numpy as np
import matplotlib.pyplot as plt

# Core control libraries
import control  # python-control: pip install control
from control import TransferFunction, step_response, impulse_response, forced_response, bode_plot

# Parameters of a typical servo joint (robotics-like)
omega_n = 20.0   # rad/s
zeta    = 0.4    # damping ratio

# Define transfer function G(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
num = [omega_n**2]
den = [1.0, 2.0*zeta*omega_n, omega_n**2]
G = TransferFunction(num, den)

# 1) Automated step and impulse responses
t_step, y_step = step_response(G)
t_imp,  y_imp  = impulse_response(G)

# 2) Forced response to a sinusoidal input u(t) = sin(omega_in * t)
omega_in = 5.0  # rad/s
t = np.linspace(0.0, 4.0, 2000)
u = np.sin(omega_in * t)
t_forced, y_forced, x_forced = forced_response(G, T=t, U=u)

# 3) Frequency response (Bode data) on a log-spaced grid
w = np.logspace(-1, 2, 1000)  # 0.1 .. 100 rad/s
mag, phase, omega = control.freqresp(G, w)

# Convert magnitude to dB
mag_db = 20.0 * np.log10(np.squeeze(mag))
phase_deg = np.squeeze(np.degrees(phase))
omega = np.squeeze(omega)

# 4) Plotting (time domain)
plt.figure()
plt.plot(t_step, y_step)
plt.title("Step response")
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.grid(True)

plt.figure()
plt.plot(t_imp, y_imp)
plt.title("Impulse response")
plt.xlabel("t [s]")
plt.ylabel("g(t)")
plt.grid(True)

plt.figure()
plt.plot(t_forced, u, label="u(t) = sin(omega_in * t)")
plt.plot(t_forced, y_forced, label="y(t)")
plt.title("Forced response to sinusoidal input")
plt.xlabel("t [s]")
plt.legend()
plt.grid(True)

# 5) Plotting (Bode-like plots)
plt.figure()
plt.semilogx(omega, mag_db)
plt.title("Magnitude response |G(jw)| in dB")
plt.xlabel("omega [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.grid(True, which="both")

plt.figure()
plt.semilogx(omega, phase_deg)
plt.title("Phase response arg G(jw)")
plt.xlabel("omega [rad/s]")
plt.ylabel("Phase [deg]")
plt.grid(True, which="both")

plt.show()
