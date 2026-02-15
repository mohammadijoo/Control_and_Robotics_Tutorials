import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters for RC low-pass filter (sensor anti-alias filter)
R = 10_000.0      # 10 kOhm
C = 1e-6          # 1 microfarad
RC = R * C

# Parameters for op-amp integrator (e.g., analog velocity estimate)
R_int = 100_000.0   # 100 kOhm
C_int = 1e-6
k_int = 1.0 / (R_int * C_int)

def u_step(t, U0=1.0):
    return U0  # unit step

def rc_ode(t, vC):
    return (u_step(t) - vC) / RC

def integrator_ode(t, vo):
    # integrate the same step input
    return -k_int * u_step(t)

t_span = (0.0, 0.1)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

sol_rc = solve_ivp(rc_ode, t_span, y0=[0.0], t_eval=t_eval)
sol_int = solve_ivp(integrator_ode, t_span, y0=[0.0], t_eval=t_eval)

plt.figure()
plt.plot(sol_rc.t, sol_rc.y[0], label="RC vC(t)")
plt.plot(sol_int.t, sol_int.y[0], label="Integrator vo(t)")
plt.xlabel("time (s)")
plt.ylabel("voltage (V)")
plt.legend()
plt.title("RC low-pass and op-amp integrator responses")
plt.grid(True)
plt.show()

# In larger robotics projects, you could also represent the RC circuit as
# a transfer function using python-control:
# import control
# G_rc = control.tf([1.0], [RC, 1.0])
