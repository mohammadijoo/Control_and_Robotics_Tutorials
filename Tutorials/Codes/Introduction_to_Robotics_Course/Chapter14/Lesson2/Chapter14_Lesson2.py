import numpy as np

T = 0.01      # sampling period [s]
T_end = 10.0  # simulation horizon
k_gain = 1.0  # autonomous feedback gain
alpha = 0.4   # human authority in [0,1)
U_max = 2.0   # actuator saturation

N = int(T_end / T)
t = np.linspace(0.0, T_end, N + 1)
x = np.zeros(N + 1)

def human_command(time):
    # Simple piecewise-constant input: push right for 0-5s, left for 5-10s
    if time < 5.0:
        return 1.0
    else:
        return -1.0

for k in range(N):
    u_h = human_command(t[k])
    u_a = -k_gain * x[k]
    u = alpha * u_h + (1.0 - alpha) * u_a
    # Saturation
    u_sat = max(-U_max, min(U_max, u))
    # Euler forward integration of x_dot = u
    x[k + 1] = x[k] + T * u_sat

# x and t now contain the simulated trajectory under shared autonomy
print("Final position:", x[-1])
      
