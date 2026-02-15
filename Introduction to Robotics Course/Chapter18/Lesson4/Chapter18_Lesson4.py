import numpy as np

# Simulation parameters
dt = 0.001
T = 5.0
steps = int(T / dt)

# True friction coefficient
a_true = 2.0

# Controller and learning gains
K = 10.0           # feedback gain
gamma = 5.0        # learning rate for a_hat

# Desired reference velocity (step)
def x_ref(t):
    return 1.0 if t > 0.5 else 0.0

x = 0.0            # initial velocity
a_hat = 0.0        # initial estimate of friction

xs = []
xrefs = []
ahat_hist = []

for k in range(steps):
    t = k * dt
    xr = x_ref(t)
    e = x - xr

    # Control law: u = -K e + a_hat * x
    u_fb = -K * e
    u_ff = a_hat * x
    u = u_fb + u_ff

    # True dynamics: x_{k+1} = x_k + dt * (u - a_true * x_k)
    x_dot = u - a_true * x
    x = x + dt * x_dot

    # Parameter adaptation: a_hat_{k+1} = a_hat_k - gamma * e * x
    a_hat = a_hat - gamma * e * x * dt  # dt can be absorbed into gamma

    xs.append(x)
    xrefs.append(xr)
    ahat_hist.append(a_hat)

# Optional: plot results (requires matplotlib)
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    tgrid = np.arange(steps) * dt
    plt.figure()
    plt.plot(tgrid, xs, label="x")
    plt.plot(tgrid, xrefs, "--", label="x_ref")
    plt.xlabel("time [s]")
    plt.ylabel("velocity")
    plt.legend()

    plt.figure()
    plt.plot(tgrid, ahat_hist, label="a_hat")
    plt.axhline(a_true, linestyle="--", label="a_true")
    plt.xlabel("time [s]")
    plt.ylabel("friction estimate")
    plt.legend()
    plt.show()
      
