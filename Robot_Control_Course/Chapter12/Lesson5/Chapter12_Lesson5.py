
import time
import math
import numpy as np

# Controller parameters
Ts = 0.002  # nominal sampling time [s]
Kp = 50.0
Kd = 2.0

# Reference trajectory: simple sinusoid
def q_ref(t):
    qd = 0.5 * math.sin(2.0 * math.pi * 0.5 * t)  # 0.5 rad amplitude, 0.5 Hz
    dqd = 0.5 * 2.0 * math.pi * 0.5 * math.cos(2.0 * math.pi * 0.5 * t)
    return qd, dqd

# Robot I/O (to be implemented for your platform)
def read_joint():
    """
    Return (q, dq) for the controlled joint.
    Connect this to your low-level interface or simulator.
    """
    # Example dummy implementation:
    # return sim.get_joint_state()
    raise NotImplementedError

def write_torque(tau):
    """
    Send torque tau to the actuator.
    """
    # Example dummy implementation:
    # sim.set_joint_torque(tau)
    raise NotImplementedError

# Logging arrays
log_t = []
log_Tk = []
log_jitter = []
log_q = []
log_qd = []
log_e = []

t_final = 5.0  # run 5 seconds
t0 = time.perf_counter()
last_time = t0
next_time = t0 + Ts

e_prev = 0.0

while True:
    now = time.perf_counter()
    t = now - t0
    if t >= t_final:
        break

    # Measured period and jitter
    Tk = now - last_time
    jitter = Tk - Ts
    last_time = now

    # Read current joint state
    q, dq = read_joint()

    # Reference at current time
    qd, dqd = q_ref(t)

    # PD control in discrete time
    e = qd - q
    de = (e - e_prev) / max(Tk, 1e-6)  # use measured Tk in derivative
    e_prev = e

    tau = Kp * e + Kd * de

    # Send control
    write_torque(tau)

    # Log signals
    log_t.append(t)
    log_Tk.append(Tk)
    log_jitter.append(jitter)
    log_q.append(q)
    log_qd.append(qd)
    log_e.append(e)

    # Sleep until next nominal sample time
    next_time += Ts
    sleep_time = next_time - time.perf_counter()
    if sleep_time > 0.0:
        time.sleep(sleep_time)

# After the loop, analyze logs:
# - Plot Tk vs t
# - Plot jitter vs t
# - Plot tracking error vs t
