import numpy as np
import matplotlib.pyplot as plt

# Load log: columns t, r, y, u
data = np.loadtxt("log.csv", delimiter=",", skiprows=1)
t = data[:, 0]
r = data[:, 1]
y = data[:, 2]
u = data[:, 3]

dt = np.mean(np.diff(t))
e = r - y

J_ISE = np.sum(e**2) * dt
e_RMS = np.sqrt(np.mean(e**2))
e_inf = np.max(np.abs(e))

print(f"ISE = {J_ISE:.3f}")
print(f"RMS error = {e_RMS:.3f}")
print(f"Max error = {e_inf:.3f}")

# Example thresholds
RMS_max = 0.02
e_inf_max = 0.05

if e_RMS <= RMS_max and e_inf <= e_inf_max:
    print("Tracking requirements satisfied.")
else:
    print("Tracking requirements NOT satisfied.")

# Simple plot for presentation
plt.figure()
plt.plot(t, r, label="reference")
plt.plot(t, y, label="output")
plt.xlabel("time [s]")
plt.ylabel("position")
plt.legend()
plt.grid(True)
plt.show()
      
