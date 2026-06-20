import numpy as np
import matplotlib.pyplot as plt

# DC motor parameters (SI)
R = 1.2        # ohm
L = 2.0e-3     # H
ke = 0.08      # V*s/rad
kt = ke        # N*m/A (proved)
J = 5.0e-4     # kg*m^2
b = 1.0e-3     # N*m*s/rad

dt = 1e-4
T_end = 0.5
N = int(T_end/dt)

# PID gains for speed control
Kp, Ki, Kd = 0.4, 30.0, 0.0
omega_ref = 100.0  # rad/s

i = 0.0
omega = 0.0
e_int = 0.0
e_prev = 0.0

omega_log = np.zeros(N)
v_log = np.zeros(N)
t_log = np.arange(N)*dt

for k in range(N):
    # PID on speed
    e = omega_ref - omega
    e_int += e*dt
    e_der = (e - e_prev)/dt
    v = Kp*e + Ki*e_int + Kd*e_der
    e_prev = e

    # Electrical update: di/dt = (v - R i - ke omega)/L
    di = (v - R*i - ke*omega)/L
    i += di*dt

    # Mechanical update: domega/dt = (kt i - b omega)/J
    domega = (kt*i - b*omega)/J
    omega += domega*dt

    omega_log[k] = omega
    v_log[k] = v

plt.figure()
plt.plot(t_log, omega_log)
plt.xlabel("time (s)")
plt.ylabel("speed omega (rad/s)")
plt.title("PID Speed Control of a DC Motor")
plt.grid(True)
plt.show()
