
import numpy as np

# Base (fast) sampling period and multi-rate factor
T_fast = 0.001        # 1 kHz torque or velocity loop
m = 10                # outer loop at 100 Hz
T_slow = m * T_fast

# Simple double-integrator discrete model: x = [q, qdot]
# x[k+1] = A_f x[k] + B_f u[k]
A_f = np.array([[1.0, T_fast],
                [0.0, 1.0]])
B_f = np.array([[0.5 * T_fast**2],
                [T_fast]])

# Fast inner feedback (e.g. on velocity) and slow outer loop on position
K_fast = np.array([[0.0, 5.0]])       # simple velocity feedback
K_slow = 50.0                         # proportional position gain

# Reference trajectory for joint position
def q_ref(t):
    # simple step reference
    return 0.5 if t > 0.1 else 0.0

tf = 1.0
N_steps = int(tf / T_fast)
x = np.zeros((2,))     # initial state [q, qdot]
v_slow = 0.0           # outer-loop "virtual torque" command

q_hist = []
t_hist = []

for k in range(N_steps):
    t = k * T_fast

    # Slow outer loop every m fast steps
    if k % m == 0:
        q_des = q_ref(t)
        e_q = q_des - x[0]
        v_slow = K_slow * e_q

    # Fast inner loop (velocity feedback)
    u = v_slow - float(K_fast @ x)

    # Plant update
    x = A_f @ x + (B_f.flatten() * u)

    q_hist.append(x[0])
    t_hist.append(t)

# q_hist and t_hist now contain the simulated joint trajectory
print("Final joint position:", q_hist[-1])
