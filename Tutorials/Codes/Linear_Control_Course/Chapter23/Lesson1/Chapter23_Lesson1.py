import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control, widely used in robotics/control

# Nominal parameters (e.g. small robot joint)
J0 = 0.01  # kg m^2
b0 = 0.05  # N m s/rad

def joint_tf(J, b):
    # G(s) = 1 / (J s^2 + b s)
    num = [1.0]
    den = [J, b, 0.0]
    return ct.TransferFunction(num, den)

G_nom = joint_tf(J0, b0)

# Generate random parametric variations (±20%)
rng = np.random.default_rng(seed=1)
n_samples = 8
Js = J0 * (1.0 + 0.4 * (rng.random(n_samples) - 0.5))
bs = b0 * (1.0 + 0.4 * (rng.random(n_samples) - 0.5))

t = np.linspace(0, 4.0, 500)
plt.figure()
for J, b in zip(Js, bs):
    G = joint_tf(J, b)
    tout, yout = ct.step_response(G, t)
    plt.plot(tout, yout, alpha=0.6)

# Nominal response highlighted
tout_nom, y_nom = ct.step_response(G_nom, t)
plt.plot(tout_nom, y_nom, linestyle="--", linewidth=2)

plt.xlabel("time [s]")
plt.ylabel("joint angle response to unit step torque")
plt.title("Effect of parametric uncertainty on robot joint step response")
plt.grid(True)
plt.show()
