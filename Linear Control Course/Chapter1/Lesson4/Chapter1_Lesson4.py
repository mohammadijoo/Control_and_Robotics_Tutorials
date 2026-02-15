import numpy as np

def simulate_first_order(a=1.0, b=1.0, k=5.0, r=1.0,
                         dt=0.001, t_final=5.0):
    """
    Simulate closed-loop system:
        dx/dt = -(a + b k) x + b k r
        y = x,  u = k (r - x)

    In robotics, similar logic would run in a low-level controller node
    (e.g. using ROS topics instead of printing to console).
    """
    n_steps = int(t_final / dt)
    x = 0.0
    t = np.linspace(0.0, t_final, n_steps)
    x_hist = np.zeros(n_steps)
    u_hist = np.zeros(n_steps)

    for i in range(n_steps):
        e = r - x
        u = k * e
        dx = -(a + b * k) * x + b * k * r
        x = x + dt * dx

        x_hist[i] = x
        u_hist[i] = u

    # Steady-state values (approximate)
    x_ss = x_hist[-1]
    e_ss = r - x_ss

    # Approximate 2% settling time
    tol = 0.02 * abs(x_ss) if x_ss != 0.0 else 0.02
    t_settle = t_final
    for i in range(n_steps):
        if np.all(np.abs(x_hist[i:] - x_ss) <= tol):
            t_settle = t[i]
            break

    print("Approx steady-state output y_ss =", x_ss)
    print("Approx steady-state error e_ss =", e_ss)
    print("Approx settling time (2%)     =", t_settle)
    print("Peak control effort max|u|    =", np.max(np.abs(u_hist)))

if __name__ == "__main__":
    simulate_first_order(a=1.0, b=1.0, k=5.0, r=1.0)
