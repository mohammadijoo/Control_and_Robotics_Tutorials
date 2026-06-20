import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control: pip install control

# Robot joint parameters (simplified)
J = 0.01   # inertia
B = 0.1    # viscous friction

# Plant: 1 / (J s^2 + B s)
G = ct.tf([1.0], [J, B, 0.0])

# PI controller without rolloff: C_low(s) = Kp + Ki/s
Kp = 30.0
Ki = 80.0
s = ct.tf([1.0, 0.0], [1.0])             # s
C_int = Ki / s                           # integral term
C_low = Kp + C_int

# First-order rolloff at wh = 200 rad/s
wh = 200.0
F1 = 1.0 / (1.0 + s / wh)

# Second-order rolloff at same corner
F2 = 1.0 / (1.0 + s / wh)**2

C_no_rolloff = C_low
C_rolloff_1  = C_low * F1
C_rolloff_2  = C_low * F2

L_no   = C_no_rolloff * G
L_ro1  = C_rolloff_1 * G
L_ro2  = C_rolloff_2 * G

# Bode magnitude of L(jw)
w = np.logspace(0, 4, 400)  # 1 to 10^4 rad/s

mag_no, phase_no, _ = ct.bode(L_no,  w, Plot=False)
mag_1,  phase_1,  _ = ct.bode(L_ro1, w, Plot=False)
mag_2,  phase_2,  _ = ct.bode(L_ro2, w, Plot=False)

plt.figure()
plt.loglog(w, mag_no, label="no rolloff")
plt.loglog(w, mag_1,  label="1st-order rolloff")
plt.loglog(w, mag_2,  label="2nd-order rolloff")
plt.xlabel("w (rad/s)")
plt.ylabel("|L(jw)|")
plt.legend()
plt.grid(True, which="both")

# Noise simulation: white noise at sensor
# Closed-loop with measurement noise
T_n_no  = -L_no  / (1.0 + L_no)
T_n_1   = -L_ro1 / (1.0 + L_ro1)
T_n_2   = -L_ro2 / (1.0 + L_ro2)

t = np.linspace(0.0, 1.0, 5000)
dt = t[1] - t[0]
sigma_n = 0.01  # noise std
n = sigma_n * np.random.randn(len(t))

_, y_n_no  = ct.forced_response(T_n_no,  T=t, U=n)
_, y_n_1   = ct.forced_response(T_n_1,   T=t, U=n)
_, y_n_2   = ct.forced_response(T_n_2,   T=t, U=n)

plt.figure()
plt.plot(t, y_n_no, label="no rolloff")
plt.plot(t, y_n_1,  label="1st-order")
plt.plot(t, y_n_2,  label="2nd-order")
plt.xlabel("time (s)")
plt.ylabel("output due to measurement noise")
plt.legend()
plt.grid(True)

plt.show()

# NOTE:
# In a robotics setting, the same C(s) would be discretized and implemented
# in middleware such as ROS using the robot joint states as feedback.
