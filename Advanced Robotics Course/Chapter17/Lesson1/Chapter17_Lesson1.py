import numpy as np

# Number of nodes in the rope
N = 10
mass = 0.05
k_spring = 50.0
damping = 0.1
g = 9.81

# Small time step due to stiffness
dt = 5e-4
steps = 20000

# Positions and velocities in 2D
x = np.zeros((N, 2))
v = np.zeros_like(x)

# Initialize rope horizontally
rest_length = 0.05
for i in range(N):
    x[i, 0] = rest_length * i
    x[i, 1] = 0.0

rest_lengths = np.full(N - 1, rest_length)

def compute_forces(x, v):
    f = np.zeros_like(x)

    # Internal springs between neighbors
    for i in range(N - 1):
        dx = x[i + 1] - x[i]
        L = np.linalg.norm(dx)
        if L > 1e-6:
            direction = dx / L
            f_spring = k_spring * (L - rest_lengths[i]) * direction
            f[i] += f_spring
            f[i + 1] -= f_spring

    # Damping and gravity
    f -= damping * v
    f[:, 1] -= g * mass
    return f

for step in range(steps):
    # External forces
    f = compute_forces(x, v)

    # Fix the first node (anchored to the table)
    f[0, :] = 0.0
    v[0, :] = 0.0
    x[0, :] = np.array([0.0, 0.0])

    # Drive the last node along a simple trajectory (robot gripper)
    t = step * dt
    target = np.array([rest_length * (N - 1),
                       0.1 * np.sin(2.0 * np.pi * t)])
    x[-1, :] = target
    v[-1, :] = np.array([0.0, 0.0])

    # Semi-implicit (symplectic) Euler integration
    a = f / mass
    v += dt * a
    x += dt * v

# In practice, this core integration loop would be embedded in a robotics
# simulator (e.g., PyBullet, MuJoCo), and coupled to robot joint-space control.
      
