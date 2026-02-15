import numpy as np
import control as ctrl  # python-control library

# SDR specifications
Mp_max = 0.10      # max overshoot (10%)
Ts_max = 1.5       # max settling time (seconds)

# Candidate closed-loop transfer function (second-order approximation)
zeta = 0.7
omega_n = 4.0
num = [omega_n**2]
den = [1.0, 2.0*zeta*omega_n, omega_n**2]
G_cl = ctrl.TransferFunction(num, den)

# Step response and performance data
t, y = ctrl.step_response(G_cl)
info = ctrl.step_info(G_cl)

Mp = info["Overshoot"] / 100.0
Ts = info["SettlingTime"]

print(f"Overshoot: {Mp:.3f}, Settling time: {Ts:.3f} s")

if Mp <= Mp_max and Ts <= Ts_max:
    print("SDR check PASSED: performance requirements satisfied.")
else:
    print("SDR check FAILED: redesign controller gains.")
      
