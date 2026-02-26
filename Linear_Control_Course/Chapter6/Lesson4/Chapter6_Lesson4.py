import numpy as np
import matplotlib.pyplot as plt

# If available, use the python-control package
# pip install control
import control as ctl

# Baseline second-order parameters
zeta = 0.4
wn   = 5.0  # rad/s

# Baseline second-order transfer function: G0(s) = wn^2 / (s^2 + 2 zeta wn s + wn^2)
num0 = [wn**2]
den0 = [1.0, 2.0*zeta*wn, wn**2]
G0 = ctl.TransferFunction(num0, den0)

# Extra pole at s = -a
a = 20.0  # fast pole
kp = a
num_p = [kp * wn**2]
den_p = np.polymul([1.0, a], den0)
Gp = ctl.TransferFunction(num_p, den_p)

# Extra left-half-plane zero at s = -z
z = 2.0
kz = 1.0 / z
num_z = [kz*wn**2, kz*z*wn**2]  # (s + z)*wn^2 / z
den_z = den0
Gz = ctl.TransferFunction(num_z, den_z)

t = np.linspace(0, 4.0, 1000)
t0, y0 = ctl.step_response(G0, T=t)
tp, yp = ctl.step_response(Gp, T=t)
tz, yz = ctl.step_response(Gz, T=t)

plt.figure()
plt.plot(t0, y0, label="Second-order")
plt.plot(tp, yp, label="With extra pole at -a")
plt.plot(tz, yz, label="With zero at -z")
plt.xlabel("Time [s]")
plt.ylabel("Response y(t)")
plt.title("Effect of Additional Pole and Zero on Step Response")
plt.grid(True)
plt.legend()
plt.show()
