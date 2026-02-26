import numpy as np

# Rolling disk parameters
R = 0.1  # radius [m]

def A_rolling_disk(q):
    """
    q = [x, y, theta, phi]
    returns A(q) in A(q) qdot = 0
    """
    x, y, theta, phi = q
    A = np.zeros((2, 4))
    # Constraint 1: xdot*cos(theta) + ydot*sin(theta) - R*phidot = 0
    A[0, 0] = np.cos(theta)      # coefficient of xdot
    A[0, 1] = np.sin(theta)      # coefficient of ydot
    A[0, 2] = 0.0               # coefficient of thetadot
    A[0, 3] = -R                # coefficient of phidot

    # Constraint 2: xdot*sin(theta) - ydot*cos(theta) = 0
    A[1, 0] = np.sin(theta)
    A[1, 1] = -np.cos(theta)
    A[1, 2] = 0.0
    A[1, 3] = 0.0
    return A

def admissible_velocity_basis(q):
    """
    Compute a basis for the nullspace of A(q), representing admissible velocities.
    """
    A = A_rolling_disk(q)
    # Use SVD-based nullspace
    U, S, Vt = np.linalg.svd(A)
    rank = np.sum(S > 1e-9)
    null_dim = Vt.shape[0] - rank
    # Last columns of Vt.T span the nullspace
    basis = Vt.T[:, rank:]
    return basis, rank, null_dim

q0 = np.array([0.0, 0.0, 0.0, 0.0])
basis, rankA, dimD = admissible_velocity_basis(q0)
print("rank(A(q0)) =", rankA)
print("dim D(q0)   =", dimD)
print("Basis for admissible velocities at q0:\n", basis)

# Example: constrain motion to "roll forward" with a scalar speed u
def rolling_disk_kinematics(q, u):
    """
    u is the control specifying forward speed along body x-axis.
    """
    x, y, theta, phi = q
    # Forward velocity v = R * phidot
    v = u
    xdot = v * np.cos(theta)
    ydot = v * np.sin(theta)
    thetadot = 0.0
    phidot = v / R
    return np.array([xdot, ydot, thetadot, phidot])

def simulate_rolling_disk(q0, dt=0.01, T=2.0, u=0.2):
    steps = int(T / dt)
    traj = np.zeros((steps + 1, len(q0)))
    traj[0] = q0
    q = q0.copy()
    for k in range(steps):
        qdot = rolling_disk_kinematics(q, u)
        q = q + dt * qdot
        traj[k + 1] = q
    return traj

traj = simulate_rolling_disk(q0)
print("Final configuration after rolling:", traj[-1])
      
