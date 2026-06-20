import numpy as np
import control as ct   # python-control library
import matplotlib.pyplot as plt

# Closed-loop parameters (from previous controller design)
omega_n = 10.0           # natural frequency [rad/s]
zeta_0  = 0.4            # existing damping
zeta_1  = 0.9            # desired effective damping for reference tracking

# Closed-loop from internal reference to output:
# T0(s) = omega_n^2 / (s^2 + 2*zeta_0*omega_n*s + omega_n^2)
T0 = ct.TransferFunction(
    [omega_n**2],
    [1.0, 2.0*zeta_0*omega_n, omega_n**2]
)

# Prefilter F(s) implementing model matching:
# F(s) = (s^2 + 2*zeta_0*omega_n*s + omega_n^2)
#        / (s^2 + 2*zeta_1*omega_n*s + omega_n^2)
F = ct.TransferFunction(
    [1.0, 2.0*zeta_0*omega_n, omega_n**2],
    [1.0, 2.0*zeta_1*omega_n, omega_n**2]
)

# Effective closed-loop from external reference to output
T_ref = F * T0

# Step responses: with and without prefilter
t = np.linspace(0.0, 3.0, 1000)
t1, y_no_pref = ct.step_response(T0, T=t)
t2, y_pref    = ct.step_response(T_ref, T=t)

plt.figure()
plt.plot(t1, y_no_pref, label="Without prefilter")
plt.plot(t2, y_pref,    label="With prefilter F(s)")
plt.xlabel("Time [s]")
plt.ylabel("Output y(t)")
plt.legend()
plt.grid(True)
plt.title("Command shaping by linear prefilter")
plt.show()

# --- Cubic reference profile generator (for robot joint motion) ---

def cubic_profile(q_f, T, t_array):
    """Return cubic point-to-point trajectory q(t) from 0 to q_f over [0, T]."""
    t = np.clip(np.array(t_array), 0.0, T)
    s = t / T
    q = 3.0*q_f*s**2 - 2.0*q_f*s**3
    return q

T_move = 1.0      # motion duration [s]
q_f    = 1.0      # target position [rad]

t_traj = np.linspace(0.0, T_move, 500)
q_traj = cubic_profile(q_f, T_move, t_traj)

plt.figure()
plt.plot(t_traj, q_traj)
plt.xlabel("Time [s]")
plt.ylabel("Reference position q(t) [rad]")
plt.title("Cubic reference trajectory")
plt.grid(True)
plt.show()

# In a robotics framework (e.g., roboticstoolbox), q_traj samples would be
# streamed as joint position setpoints to a linear PID or state-feedback servo.
