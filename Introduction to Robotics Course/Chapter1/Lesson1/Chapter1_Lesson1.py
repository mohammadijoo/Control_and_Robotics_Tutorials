
import numpy as np

class MinimalRobot:
    """
    Minimal embodied agent:
    x_r: robot internal state (here 1D position)
    x_e: environment state (here a target position)
    y  : measurement of environment relative position
    u  : actuator command (velocity)
    """
    def __init__(self, x_r0=0.0):
        self.x_r = float(x_r0)

    def sense(self, x_e):
        # measurement map y = h(x_r, x_e)
        return x_e - self.x_r

    def policy(self, y, alpha=0.0, r=0.0):
        # u(t) = alpha * r(t) + (1-alpha) * phi(y)
        phi = 0.5 * y              # simple reactive rule
        return alpha * r + (1-alpha) * phi

    def actuate(self, u, dt=0.1):
        # body dynamics: x_r_dot = u
        self.x_r += u * dt

# simulate closed-loop coupling
robot = MinimalRobot(x_r0=0.0)
x_e = 10.0  # "environment": target position

for t in range(50):
    y = robot.sense(x_e)
    u = robot.policy(y, alpha=0.2, r=0.0)  # partially autonomous
    robot.actuate(u)
print("Final robot position:", robot.x_r)
      