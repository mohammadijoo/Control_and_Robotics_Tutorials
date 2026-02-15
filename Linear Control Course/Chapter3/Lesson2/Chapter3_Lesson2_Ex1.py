import control as ct

m = 1.0
b = 0.4
k = 4.0

# Numerator and denominator of Y(s)/F(s)
num = [1.0]            # 1 in the numerator
den = [m, b, k]        # m s^2 + b s + k

G = ct.TransferFunction(num, den)

# Compute and plot the step response (force step -> displacement)
t, y = ct.step_response(G)

import matplotlib.pyplot as plt
plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("y(t) [m]")
plt.title("Step response via python-control")
plt.grid(True)
plt.show()
