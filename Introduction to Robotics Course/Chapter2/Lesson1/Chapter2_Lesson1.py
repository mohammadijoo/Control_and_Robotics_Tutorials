
import numpy as np
import matplotlib.pyplot as plt

# cam parameters
theta_r = np.pi/2        # rise ends at 90 degrees
theta_h = np.pi          # hold ends at 180 degrees
omega = 2.0              # rad/s

def cam_profile(theta):
    theta = np.mod(theta, 2*np.pi)
    if theta < theta_r:
        # quadratic rise: x = a2 * theta^2
        a2 = 1.0 / theta_r**2
        return a2 * theta**2
    elif theta < theta_h:
        return 1.0
    else:
        # quadratic return to 0 at 2pi
        b2 = 1.0 / (2*np.pi - theta_h)**2
        return b2 * (2*np.pi - theta)**2

thetas = np.linspace(0, 2*np.pi, 500)
xs = np.array([cam_profile(th) for th in thetas])
t = thetas/omega

plt.figure()
plt.plot(t, xs)
plt.xlabel("time (s)")
plt.ylabel("follower displacement x(t)")
plt.title("Cam-driven periodic motion")
plt.grid(True)
plt.show()