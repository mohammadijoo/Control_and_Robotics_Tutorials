import time
import math

# Physical and control parameters (example values)
J = 0.5          # kg m^2
B = 0.1          # N m s/rad
omega_n = 4.0    # rad/s
zeta = 0.7

Kp = J * omega_n**2
Kd = 2.0 * zeta * omega_n * J - B

tau_max = 10.0   # N m, safe torque limit
Ts = 0.005       # 5 ms control period

def sat(x, xmin, xmax):
    return max(xmin, min(xmax, x))

# Placeholder I/O functions
def read_joint_state():
    """
    Returns (theta, theta_dot) in radians and rad/s.
    In a real system, this reads encoders and a velocity estimator.
    """
    # TODO: replace with actual sensor reading
    return 0.0, 0.0

def write_joint_torque(tau_cmd):
    """
    Sends torque command tau_cmd (N m) to the motor driver.
    """
    # TODO: replace with actual actuator interface
    pass

# Simple time-varying reference: sinusoidal trajectory
def desired_motion(t):
    theta_d = 0.4 * math.sin(0.5 * t)   # rad
    theta_d_dot = 0.4 * 0.5 * math.cos(0.5 * t)
    return theta_d, theta_d_dot

e_prev = 0.0

while True:
    t_now = time.time()
    theta, theta_dot = read_joint_state()
    theta_d, theta_d_dot = desired_motion(t_now)

    e = theta_d - theta
    # naive derivative of error (could be low-pass filtered in practice)
    e_dot = theta_d_dot - theta_dot

    tau_cmd = Kp * e + Kd * e_dot
    tau_cmd = sat(tau_cmd, -tau_max, tau_max)

    write_joint_torque(tau_cmd)
    time.sleep(Ts)
      
