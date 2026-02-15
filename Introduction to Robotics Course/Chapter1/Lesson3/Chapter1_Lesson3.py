
import numpy as np

def simulate_shared_autonomy(x0=2.0, k=1.5, alpha=0.7, T=5.0, dt=0.01):
    n_steps = int(T/dt)
    x = x0
    xs, us = [x], []

    for t in range(n_steps):
        u_A = -k * x                # autonomous stabilizing feedback
        u_H = 0.5*np.sin(2*np.pi*t*dt)  # example bounded human input
        u   = alpha*u_A + (1-alpha)*u_H

        x += dt*u                   # plant: x_dot = u
        xs.append(x)
        us.append(u)
    return np.array(xs), np.array(us)

xs, us = simulate_shared_autonomy()
print("Final state:", xs[-1])
      