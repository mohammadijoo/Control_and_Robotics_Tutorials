import numpy as np

def sat(u, u_max):
    return np.clip(u, -u_max, u_max)

# Parameters
k = 5.0            # feedback gain
D = 3.0            # constant disturbance
u_max = 1.0        # actuator limit
x0 = 0.0           # initial condition
t_final = 20.0
dt = 0.001

N = int(t_final / dt)
t = np.linspace(0.0, t_final, N + 1)
x = np.zeros(N + 1)
u_cmd = np.zeros(N + 1)
u_applied = np.zeros(N + 1)

x[0] = x0

for n in range(N):
    # Linear controller (commanded input)
    u_cmd[n] = -k * x[n]
    # Saturated actuator
    u_applied[n] = sat(u_cmd[n], u_max)
    # First-order dynamics: x_dot = -x + u + D
    x_dot = -x[n] + u_applied[n] + D
    x[n + 1] = x[n] + dt * x_dot

# final command value
u_cmd[N] = -k * x[N]
u_applied[N] = sat(u_cmd[N], u_max)

# Example: plotting (requires matplotlib)
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(t, x)
    axs[0].set_ylabel("y(t) = x(t)")
    axs[1].plot(t, u_applied, label="u_applied")
    axs[1].plot(t, u_cmd, "--", label="u_cmd (linear)")
    axs[1].set_ylabel("u(t)")
    axs[1].set_xlabel("time [s]")
    axs[1].legend()
    plt.show()
