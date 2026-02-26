import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
omega_n = 1.0
zeta = 0.2

# Controller parameters
K = 8.0
u_max = 2.0

def sat(u, u_max):
    return np.clip(u, -u_max, u_max)

def closed_loop_step(t_final=20.0, dt=1e-3):
    n_steps = int(t_final / dt)
    t = np.linspace(0.0, t_final, n_steps + 1)
    x = np.zeros((2, n_steps + 1))  # [x1; x2]
    y = np.zeros(n_steps + 1)
    u = np.zeros(n_steps + 1)

    for k in range(n_steps):
        r = 1.0  # unit step
        y[k] = x[0, k]
        e = r - y[k]
        v = K * e
        u[k] = sat(v, u_max)

        # state derivatives
        x1, x2 = x[:, k]
        dx1 = x2
        dx2 = -2.0 * zeta * omega_n * x2 - omega_n**2 * x1 + omega_n**2 * u[k]

        # Euler integration
        x[0, k+1] = x1 + dt * dx1
        x[1, k+1] = x2 + dt * dx2

    # final output at last step
    y[-1] = x[0, -1]
    e = 1.0 - y
    return t, y, u, e

if __name__ == "__main__":
    t, y, u, e = closed_loop_step()
    fig1, ax1 = plt.subplots()
    ax1.plot(t, y)
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("y(t)")
    ax1.set_title("Output with actuator saturation")

    fig2, ax2 = plt.subplots()
    ax2.plot(t, u)
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("u(t)")
    ax2.set_title("Saturated control input")

    plt.show()
