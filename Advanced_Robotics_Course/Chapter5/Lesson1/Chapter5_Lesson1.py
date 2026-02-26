import numpy as np

def propagate_double_integrator(x0, u, dt=0.01, T_step=0.5, u_max=1.0):
    """
    x0: np.array([x, v])
    u:  scalar control (will be saturated to [-u_max, u_max])
    dt: integration step
    T_step: total propagation time
    """
    u = float(np.clip(u, -u_max, u_max))
    x = np.array(x0, dtype=float)
    trajectory = [x.copy()]

    N = int(T_step / dt)
    for k in range(N):
        # dynamics: x_dot = [v, u]
        x_dot = np.array([x[1], u])
        x = x + dt * x_dot
        trajectory.append(x.copy())
    return np.array(trajectory)

if __name__ == "__main__":
    x_start = np.array([0.0, 0.0])   # position, velocity
    u = 0.8
    traj = propagate_double_integrator(x_start, u)
    print("Final state:", traj[-1])
      
