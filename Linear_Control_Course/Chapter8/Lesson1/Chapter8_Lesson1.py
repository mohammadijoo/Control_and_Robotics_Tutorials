import numpy as np
import matplotlib.pyplot as plt
import control  # python-control library

# Plant G(s) = K / (s (s + 1))
K = 5.0
num = [K]
den = [1.0, 1.0, 0.0]  # s^2 + s + 0 = s (s + 1)
G = control.TransferFunction(num, den)

# Unity feedback closed loop
sys_cl = control.feedback(G, 1)  # 1 stands for unity feedback

# Time grid
t = np.linspace(0.0, 10.0, 2000)

# STEP INPUT: r(t) = 1
t_step, y_step = control.step_response(sys_cl, T=t)
r_step = np.ones_like(t_step)
e_step = r_step - y_step

# RAMP INPUT: r(t) = t
r_ramp = t
t_ramp, y_ramp, _ = control.forced_response(sys_cl, T=t, U=r_ramp)
e_ramp = r_ramp - y_ramp

plt.figure()
plt.plot(t_step, e_step, label="e_step(t)")
plt.plot(t_ramp, e_ramp, label="e_ramp(t)")
plt.xlabel("t [s]")
plt.ylabel("error")
plt.title("Error signals for step and ramp references")
plt.grid(True)
plt.legend()
plt.show()

# Robotics remark:
# In a ROS-based robot controller, the same logic is used inside a control loop:
# r(t) is the commanded joint trajectory, y(t) is read from joint encoders,
# and e(t) = r(t) - y(t) is passed to a controller (e.g., PID) implemented
# with rospy or ros_control.
