import numpy as np
import matplotlib.pyplot as plt

# python-control: pip install control
import control as ctrl

# Standard second-order parameters
wn = 4.0      # natural frequency (rad/s)
zeta = 0.7    # damping ratio
K = 1.0       # DC gain

# Transfer function G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
num = [K * wn**2]
den = [1.0, 2.0 * zeta * wn, wn**2]
G = ctrl.TransferFunction(num, den)

# Step response
t, y = ctrl.step_response(G)

plt.figure()
plt.plot(t, y)
plt.xlabel("time (s)")
plt.ylabel("output y(t)")
plt.title("Standard second-order step response (Python)")
plt.grid(True)
plt.show()

# Remark:
# In robotics, G can model a single joint; G is then combined with
# trajectory generators and controllers (e.g. PID, state feedback)
# within a ROS or ROS2 control stack.
