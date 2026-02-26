import control as ctrl

# Example robotic joint plant (linearized): G(s) = 1 / (J s^2 + B s)
J = 0.01  # kg m^2
B = 0.1   # N m s/rad
s = ctrl.TransferFunction.s
G_joint = 1 / (J * s**2 + B * s)

# Proportional control with gain K
# Root locus of loop transfer function K * G_joint
ctrl.root_locus(G_joint)  # opens a matplotlib window

# To highlight damping lines or desired pole locations
import matplotlib.pyplot as plt
plt.title("Root locus of robotic joint with proportional control")
plt.show()
