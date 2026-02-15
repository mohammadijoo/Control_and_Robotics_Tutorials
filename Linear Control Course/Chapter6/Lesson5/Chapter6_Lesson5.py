import numpy as np
import matplotlib.pyplot as plt

# Use the "control" library: pip install control
import control as ctrl

# Parameters of the example
wn = 5.0      # natural frequency (rad/s)
zeta = 0.4    # damping ratio
alpha = 50.0  # fast real pole

# Full third-order transfer function:
# G3(s) = wn^2 / ((s + alpha)(s^2 + 2*zeta*wn*s + wn^2))
num_full = [wn**2]
den_full = np.polymul([1.0, alpha], [1.0, 2.0 * zeta * wn, wn**2])
G3 = ctrl.tf(num_full, den_full)

# Dominant second-order approximation:
# Keep only the complex pair and adjust DC gain.
# G2(s) = (wn^2 / alpha) / (s^2 + 2*zeta*wn*s + wn^2)
num_dom = [wn**2 / alpha]
den_dom = [1.0, 2.0 * zeta * wn, wn**2]
G2 = ctrl.tf(num_dom, den_dom)

print("Full system poles:", ctrl.pole(G3))
print("Dominant approx poles:", ctrl.pole(G2))

# Step responses for comparison
t = np.linspace(0.0, 4.0, 1000)  # time window
t1, y_full = ctrl.step_response(G3, T=t)
t2, y_dom  = ctrl.step_response(G2, T=t)

plt.figure()
plt.plot(t1, y_full, label="Full 3rd order")
plt.plot(t2, y_dom,  label="Dominant 2nd order", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.title("Dominant Pole Approximation Example")
plt.legend()
plt.grid(True)
plt.show()
