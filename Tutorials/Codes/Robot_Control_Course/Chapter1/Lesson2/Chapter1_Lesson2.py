
import numpy as np
from scipy.integrate import solve_ivp

# Optional: python-control for state-space objects
try:
    import control
except ImportError:
    control = None

# Parameters of 1-DOF joint
J = 0.02   # kg m^2
b = 0.1    # N m s/rad
k = 1.0    # N m/rad

def joint_dynamics(t, x, u_func):
    """
    x = [q, q_dot]
    u_func: function of time returning tau(t)
    """
    q, q_dot = x
    tau = u_func(t)

    q_dot_dot = (tau - b*q_dot - k*q) / J
    return np.array([q_dot, q_dot_dot])

def step_torque(t):
    # Constant 1 N m torque
    return 1.0

# Simulate
t_span = (0.0, 5.0)
x0 = np.array([0.0, 0.0])

sol = solve_ivp(
    fun=lambda t, x: joint_dynamics(t, x, step_torque),
    t_span=t_span, y0=x0, max_step=1e-3
)

# Optionally create an LTI state-space object for linear analysis
A = np.array([[0.0,     1.0],
              [-k/J, -b/J]])
B = np.array([[0.0],
              [1.0/J]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

if control is not None:
    ss_sys = control.ss(A, B, C, D)
    # ss_sys can be used later for linear control design, frequency analysis, etc.
