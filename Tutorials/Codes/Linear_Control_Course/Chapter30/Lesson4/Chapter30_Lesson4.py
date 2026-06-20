import numpy as np
import matplotlib.pyplot as plt

# python-control library: pip install control
import control as ctl

# Laplace variable
s = ctl.TransferFunction.s

# Fast subsystem P1(s) and slow subsystem P2(s)
P1 = 1 / (0.05 * s + 1)     # time constant 0.05 s (fast)
P2 = 1 / (s * (0.5 * s + 1))  # second-order-ish slow dynamics

# Inner PI controller C_in(s) = Kp_in + Ki_in / s
Kp_in = 5.0
Ki_in = 50.0
C_in = Kp_in + Ki_in / s

# Inner closed-loop complementary sensitivity T_in(s)
T_in = ctl.feedback(C_in * P1, 1)

# Equivalent plant for outer loop
P_eq = T_in * P2

# Outer PI controller C_out(s)
Kp_out = 2.0
Ki_out = 1.0
C_out = Kp_out + Ki_out / s

# Overall closed-loop (reference r to output y)
L_tot = C_out * P_eq
T_tot = ctl.feedback(L_tot, 1)

# Compare with ideal outer loop assuming T_in(s) = 1
P_ideal = P2
L_ideal = C_out * P_ideal
T_ideal = ctl.feedback(L_ideal, 1)

t = np.linspace(0, 5, 1000)
t1, y_actual = ctl.step_response(T_tot, T=t)
t2, y_ideal  = ctl.step_response(T_ideal, T=t)

plt.figure()
plt.plot(t1, y_actual, label="Two-loop actual")
plt.plot(t2, y_ideal, "--", label="Outer loop with ideal inner unity block")
plt.xlabel("Time [s]")
plt.ylabel("Output y(t)")
plt.grid(True)
plt.legend()
plt.title("Role of fast inner loop: near-unity equivalent plant for outer loop")
plt.show()
