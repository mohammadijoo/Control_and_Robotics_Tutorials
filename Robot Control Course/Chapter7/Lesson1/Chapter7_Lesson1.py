
import numpy as np

# Nominal parameters
theta_nom = np.array([0.5, 0.05, 1.0])  # [I_hat, b_hat, (m g l)_hat]
delta_bounds = np.array([0.1, 0.02, 0.2])  # max deviations |Delta theta_i|

def sample_theta():
    """Sample parameters within a box around theta_nom."""
    return theta_nom + (2.0 * np.random.rand(3) - 1.0) * delta_bounds

def disturbance(t):
    """Matched disturbance: bounded torque ripple."""
    return 0.2 * np.sin(5.0 * t)  # ||d(t)||_inf <= 0.2

def joint_dynamics(t, x, u, theta):
    """
    x = [q, qdot]
    theta = [I, b, mgl]
    """
    q, qdot = x
    I, b, mgl = theta

    # Unmodeled dynamics: crude Coulomb friction + small unmodeled torque
    fc = 0.05 * np.sign(qdot)  # unmodeled friction
    wu = 0.05 * np.sin(20.0 * t)  # high-frequency unmodeled mode

    # True dynamics (one-step Euler model)
    qddot = (u - b * qdot - mgl * np.sin(q) - fc + disturbance(t) + wu) / I
    return np.array([qdot, qddot])

def pd_control(t, x, q_ref=0.5):
    """Simple PD in joint space using nominal gains."""
    q, qdot = x
    Kp = 20.0
    Kd = 4.0
    e = q_ref - q
    edot = -qdot
    return Kp * e + Kd * edot

def simulate(T=5.0, dt=0.001, trials=5):
    N = int(T / dt)
    t_grid = np.linspace(0.0, T, N+1)
    q_hist = np.zeros((trials, N+1))
    q_ref = 0.5

    for k in range(trials):
        theta = sample_theta()
        x = np.array([0.0, 0.0])  # [q(0), qdot(0)]
        q_hist[k, 0] = x[0]

        for i in range(N):
            t = t_grid[i]
            u = pd_control(t, x, q_ref)
            dx = joint_dynamics(t, x, u, theta)
            x = x + dt * dx  # explicit Euler
            q_hist[k, i+1] = x[0]

    return t_grid, q_hist

if __name__ == "__main__":
    t, q_trajs = simulate()
    # Plot with matplotlib (not shown here); you would see a bundle of trajectories
    # corresponding to different parameter samples and disturbances.
