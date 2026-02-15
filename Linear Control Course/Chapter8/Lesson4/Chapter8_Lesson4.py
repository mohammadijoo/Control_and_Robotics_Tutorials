import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Plant model: approximated joint actuator (continuous time)
s = ct.TransferFunction.s
G = 5 / (s + 3)

# PI controller from the analytic design
Kp = 1.0
Ki = 4.0
C = Kp + Ki / s  # C(s) = Kp + Ki/s

L = C * G               # loop transfer function
T = ct.feedback(L, 1)   # closed-loop from r to y
S = 1 - T               # sensitivity (from r to e)

# Static error constants (step and ramp)
# Kp_static = lim_{s->0} L(s)
Kp_static = ct.dcgain(L)  # dcgain gives G(0) for proper systems

# Kv = lim_{s->0} s L(s)
Kv_tf = s * L
Kv_static = ct.dcgain(Kv_tf)

print("Kp (static) =", Kp_static)
print("Kv (static) =", Kv_static)
print("Predicted ramp error (unit ramp) =", 1.0 / Kv_static)

# Numerical simulation for step and ramp
t = np.linspace(0, 10, 1000)

# Step response (unit step)
t_step, y_step = ct.step_response(T, T=t)
e_step = 1.0 - y_step

# Ramp response: r(t) = t for t >= 0
r_ramp = t
t_ramp, y_ramp, x_ramp = ct.forced_response(T, T=t, U=r_ramp)
e_ramp = r_ramp - y_ramp

print("Approximate steady-state step error =", e_step[-1])
print("Approximate steady-state ramp error =", e_ramp[-1])

# Plot ramp tracking
plt.figure()
plt.plot(t_ramp, r_ramp, label="r(t) = ramp")
plt.plot(t_ramp, y_ramp, label="y(t)")
plt.xlabel("Time [s]")
plt.ylabel("Position")
plt.legend()
plt.title("Ramp tracking with PI controller")
plt.grid(True)
plt.show()
