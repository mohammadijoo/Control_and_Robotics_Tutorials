import numpy as np
import control as ctl  # python-control library (common in robotics courses)

# 1. Define plant G(s) = 1 / (s (s + 1))
s = ctl.TransferFunction.s
G = 1 / (s * (s + 1))

# 2. Choose crossover frequency and compute K analytically
omega_gc = 0.7
# |G(j w)| = 1 / (w * sqrt(1 + w^2))
mag_G = 1.0 / (omega_gc * np.sqrt(1.0 + omega_gc**2))
K = 1.0 / mag_G  # unity gain at w = omega_gc

C = K  # proportional controller
L = C * G

# 3. Verify margins
gm, pm, w_gc_num, w_pc_num = ctl.margin(L)
print("Designed K =", K)
print("Numerical PM (deg) =", pm)
print("Numerical GM (abs) =", gm, "GM (dB) =", 20 * np.log10(gm))
print("Numerical w_gc =", w_gc_num)

# 4. Closed-loop response (for a step position command)
T_cl = ctl.feedback(L, 1)  # unity feedback

t = np.linspace(0, 20, 1000)
t_out, y_out = ctl.step_response(T_cl, T=t)

# (Plotting omitted here; in practice, use matplotlib to inspect step response.)

# 5. Robot-related remark:
# In a typical ROS-based robot joint controller, the plant G(s) would be obtained
# from the rigid-body dynamics and actuator model. The same margin-based design
# applies to each low-level joint loop before higher-level motion planning is added.
