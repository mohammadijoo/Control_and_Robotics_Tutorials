import numpy as np

# Time step
dt = 0.1

def f(x, u):
    """
    Stochastic unicycle motion model without noise.
    x: np.array shape (3,), [x, y, phi]
    u: np.array shape (2,), [v, omega]
    """
    px, py, phi = x
    v, omega = u
    return np.array([
        px + v * np.cos(phi) * dt,
        py + v * np.sin(phi) * dt,
        phi + omega * dt
    ])

# Process noise covariance (x, y, phi)
Q = np.diag([0.01**2, 0.01**2, np.deg2rad(1.0)**2])

# Measurement model: range and bearing to a known landmark at (lx, ly)
def h(x, landmark):
    px, py, phi = x
    lx, ly = landmark
    dx = lx - px
    dy = ly - py
    rng = np.hypot(dx, dy)
    bearing = np.arctan2(dy, dx) - phi
    return np.array([rng, bearing])

R = np.diag([0.05**2, np.deg2rad(2.0)**2])

rng = np.random.default_rng(seed=42)

def sample_process_noise():
    return rng.multivariate_normal(mean=np.zeros(3), cov=Q)

def sample_measurement_noise():
    return rng.multivariate_normal(mean=np.zeros(2), cov=R)

# Monte Carlo propagation
N = 10000
x0 = np.array([0.0, 0.0, 0.0])
u = np.array([1.0, 0.2])  # constant command
landmark = np.array([5.0, 0.0])

particles = np.repeat(x0[None, :], N, axis=0)
for t in range(20):
    for i in range(N):
        x_nom = f(particles[i], u)
        w = sample_process_noise()
        particles[i] = x_nom + w

# Empirical mean and covariance of state after 20 steps
mu_hat = particles.mean(axis=0)
Sigma_hat = np.cov(particles.T)

print("Estimated mean:", mu_hat)
print("Estimated covariance:\n", Sigma_hat)

# Sample noisy measurement from one particle
x_sample = particles[0]
y_nom = h(x_sample, landmark)
v = sample_measurement_noise()
y = y_nom + v
print("Noisy measurement (range, bearing):", y)

# Example: using the Robotics Toolbox for Python for more complex robots
# (Peter Corke, roboticstoolbox-python)
# from roboticstoolbox import DHRobot, RevoluteDH
# Define a simple manipulator and sample link parameter uncertainties, etc.
      
