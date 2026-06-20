
import numpy as np

# DC motor parameters (example)
R = 2.0       # ohm
L = 0.5e-3    # H
J = 1.0e-3    # kg*m^2
b = 1.0e-4    # N*m*s/rad
ke = 0.05     # V*s/rad
kt = 0.05     # N*m/A

# Encoder
N = 4096
Delta_theta = 2*np.pi/N

# Discrete-time control
Ts = 1e-3
Kp = 2.0
Kd = 0.02
theta_ref = 1.0  # rad step reference

T_end = 0.5
steps = int(T_end/Ts)

# States: [theta, omega, current]
x = np.zeros(3)
theta_log = []
u_log = []

def quantize(theta):
    return Delta_theta * np.round(theta/Delta_theta)

for k in range(steps):
    theta, omega, i = x

    # Sensor with quantization and noise
    theta_q = quantize(theta)
    y = theta_q + np.random.normal(0, Delta_theta/np.sqrt(12))

    # Discrete PD controller on position
    e = theta_ref - y
    u = Kp*e - Kd*omega  # voltage command

    # Saturate voltage (actuator limit)
    u = np.clip(u, -12.0, 12.0)

    # Continuous motor dynamics integrated by Euler
    di = (u - R*i - ke*omega)/L
    domega = (kt*i - b*omega)/J
    dtheta = omega

    x = x + Ts*np.array([dtheta, domega, di])

    theta_log.append(theta)
    u_log.append(u)

print("Final theta:", theta_log[-1])
      