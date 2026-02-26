import time
import numpy as np

# Controller gains (tuned in simulation)
Kp = 8.0
Kd = 1.5

Ts = 0.01  # 10 ms sampling

# Desired trajectory (constant reference for simplicity)
q_ref = 1.0      # radians
dq_ref = 0.0     # rad/s

def counts_to_rad(counts, N_counts):
    return 2.0 * np.pi * counts / N_counts

def pwm_from_torque(u_phys, u_min, u_max, d_min, d_max):
    # Clip torque command
    u = max(min(u_phys, u_max), u_min)
    # Affine mapping to duty cycle
    return (u - u_min) * (d_max - d_min) / (u_max - u_min) + d_min

# Hardware-specific stubs (replace with real API, e.g. ROS2, vendor SDK)
def read_encoder_counts():
    # TODO: read from real encoder
    return 0

def read_velocity_estimate():
    # TODO: could be estimated via finite differences or a velocity sensor
    return 0.0

def write_pwm_command(duty):
    # TODO: write to motor driver
    pass

N_counts = 4096
u_min, u_max = -2.0, 2.0    # Nm
d_min, d_max = 0.1, 0.9     # duty cycle range

next_time = time.time()
while True:
    # Wait until next sample
    now = time.time()
    if now < next_time:
        time.sleep(next_time - now)
    next_time += Ts

    # Read sensors in raw units and convert
    c = read_encoder_counts()
    q = counts_to_rad(c, N_counts)
    dq = read_velocity_estimate()

    # PD control in physical units
    e = q_ref - q
    de = dq_ref - dq
    u = Kp * e + Kd * de

    # Convert to PWM and send
    duty = pwm_from_torque(u, u_min, u_max, d_min, d_max)
    write_pwm_command(duty)
      
