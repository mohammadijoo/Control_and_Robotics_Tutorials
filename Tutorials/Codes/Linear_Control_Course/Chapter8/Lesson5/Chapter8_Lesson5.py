import numpy as np

def simulate_first_order(K, t_final=10.0, dt=1e-3):
    """
    Plant: xdot = -x + u, y = x
    Controller: u = K * (r - y), with unit step reference r(t) = 1.
    Unity feedback.
    """
    n_steps = int(t_final / dt)
    t = np.linspace(0.0, t_final, n_steps + 1)
    x = 0.0
    y = np.zeros_like(t)
    u = np.zeros_like(t)
    r = 1.0

    for k in range(n_steps + 1):
        e = r - x   # x is current output
        u[k] = K * e
        y[k] = x
        # Euler integration of xdot = -x + u
        xdot = -x + u[k]
        x = x + dt * xdot

    y_ss = y[-1]
    ess = r - y_ss
    # approximate two percent settling time
    tol = 0.02 * abs(y_ss)
    ts = t[-1]
    for k in range(len(t)):
        if np.all(np.abs(y[k:] - y_ss) <= tol):
            ts = t[k]
            break
    umax = np.max(np.abs(u))
    return t, y, u, ess, ts, umax

if __name__ == "__main__":
    for K in [1.0, 2.0, 5.0, 10.0]:
        t, y, u, ess, ts, umax = simulate_first_order(K)
        print(f"K = {K:4.1f} | ess = {ess:.4f}, ts = {ts:.3f}, umax = {umax:.3f}")
