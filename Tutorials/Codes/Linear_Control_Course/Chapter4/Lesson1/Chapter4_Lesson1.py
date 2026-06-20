import control as ctl  # python-control library
import numpy as np

K = 2.0   # static gain
T = 0.5   # time constant

# Transfer function G(s) = K / (T s + 1)
num = [K]
den = [T, 1.0]
G = ctl.TransferFunction(num, den)
print("G(s) =", G)

# Example input: unit step response (relevant for servo motion in robotics)
t = np.linspace(0, 5, 500)
t, y = ctl.step_response(G, T=t)

# For robotic applications, this G(s) might approximate a DC motor speed loop
# in a simulation environment (e.g., within a ROS-based testbench).
