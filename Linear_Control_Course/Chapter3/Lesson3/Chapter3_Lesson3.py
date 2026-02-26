import numpy as np
from scipy.integrate import solve_ivp

# Rotational + electrical parameters
J = 0.01    # kg m^2
b = 0.1     # N m s/rad
R_a = 1.0   # ohm
L_a = 0.5   # H
K_t = 0.01  # N m/A
K_e = 0.01  # V s/rad

def motor_ode(t, x, v_func, T_load_func):
    """
    x = [theta, omega, i_a]
    """
    theta, omega, i_a = x
    v_a = v_func(t)
    T_load = T_load_func(t)

    dtheta = omega
    domega = (K_t * i_a - b * omega - T_load) / J
    di_a = (v_a - R_a * i_a - K_e * omega) / L_a
    return [dtheta, domega, di_a]

# Step input voltage and zero load
v_step = lambda t: 24.0
T_load_zero = lambda t: 0.0

t_span = (0.0, 2.0)
x0 = [0.0, 0.0, 0.0]

sol = solve_ivp(
    lambda t, x: motor_ode(t, x, v_step, T_load_zero),
    t_span,
    x0,
    max_step=1e-3
)

time = sol.t
theta = sol.y[0]
omega = sol.y[1]
i_a = sol.y[2]

# In robotics/ROS, a similar ODE would be evaluated inside a control loop
# running at a fixed sampling period, e.g., using rclpy or rclcpp.
