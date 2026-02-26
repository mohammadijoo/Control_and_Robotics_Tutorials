import numpy as np

# Link lengths
l1 = 1.0
l2 = 0.8

def q_traj(t, q10, q20, a1, a2, b1, b2):
    """
    Simple polynomial joint trajectories:
      q1(t) = q10 + a1 * t + b1 * t**2
      q2(t) = q20 + a2 * t + b2 * t**2
    """
    q1 = q10 + a1 * t + b1 * t**2
    q2 = q20 + a2 * t + b2 * t**2
    return np.vstack((q1, q2))  # shape (2, len(t))

def fk_2r(q):
    """
    Forward kinematics for planar 2R:
      q: array of shape (2, N) or (2,)
      returns x, y arrays for each column of q
    """
    q = np.atleast_2d(q)
    q1 = q[0, :]
    q2 = q[1, :]
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y

# Time discretization
T = 2.0
N = 200
t = np.linspace(0.0, T, N + 1)

# Parameters of joint trajectory
q10, q20 = 0.0, 0.5
a1, a2 = 0.8, -0.3
b1, b2 = -0.2, 0.15

# Evaluate joint-space trajectory
q = q_traj(t, q10, q20, a1, a2, b1, b2)  # shape (2, N+1)

# Map to task-space
x, y = fk_2r(q)

# x, y now describe the end-effector path in the plane as functions of t
# Example: save trajectory as (t, x, y)
traj = np.vstack((t, x, y)).T

# (Optional) quick plot if running in a Python environment with matplotlib:
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(x, y, "-")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("End-effector trajectory induced by q(t)")
    plt.axis("equal")
    plt.show()
      
