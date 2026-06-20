import numpy as np

# SEA parameters
Jm, Jl = 0.01, 0.05     # inertias
bm, bl = 0.02, 0.05     # damping
N = 50                  # gear ratio
ks = 200.0              # spring stiffness

# Torque PID gains
Kp, Ki, Kd = 5.0, 40.0, 0.02

dt = 1e-3
T  = 2.0
steps = int(T/dt)

# States: theta_m, omega_m, theta_l, omega_l
x = np.zeros(4)
tau_int = 0.0
prev_e = 0.0

def spring_torque(theta_m, theta_l):
    theta_s = N*theta_m - theta_l
    return ks*theta_s

log = []

for k in range(steps):
    t = k*dt
    tau_d = 2.0 if t > 0.2 else 0.0   # desired torque step
    tau_s = spring_torque(x[0], x[2])

    # Torque PID at motor side
    e = tau_d - tau_s
    tau_int += e*dt
    de = (e - prev_e)/dt
    prev_e = e
    tau_m = Kp*e + Ki*tau_int + Kd*de

    # Dynamics
    tau_s = spring_torque(x[0], x[2])
    theta_m, omega_m, theta_l, omega_l = x

    domega_m = (tau_m - bm*omega_m - N*tau_s) / Jm
    domega_l = (tau_s - bl*omega_l) / Jl

    # Euler integration
    omega_m += dt*domega_m
    theta_m += dt*omega_m
    omega_l += dt*domega_l
    theta_l += dt*omega_l

    x[:] = [theta_m, omega_m, theta_l, omega_l]
    log.append([t, tau_d, tau_s, theta_l])

log = np.array(log)
print("Final torque tracking error:", log[-1,1] - log[-1,2])
