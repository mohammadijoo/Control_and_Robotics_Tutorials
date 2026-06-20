import numpy as np
import matplotlib.pyplot as plt

# pip install control
import control as ct

# Plant and inner loop
s = ct.TransferFunction.s
Gp = 1 / (s + 1)     # G_p(s) = 1 / (s + 1)
k_i = 9.0
Ci = k_i
Gi_cl = ct.feedback(Ci * Gp, 1)  # inner closed loop G_i,cl(s)

# Outer PI controller
k_p = 0.5
k_I = 1.0
Co = k_p + k_I / s

# Outer closed loop T_o(s) from r to y
L_outer = Co * Gi_cl
T_outer = ct.feedback(L_outer, 1)

# Step response
t = np.linspace(0, 5, 1000)
t_out, y_out = ct.step_response(T_outer, T=t)

plt.figure()
plt.plot(t_out, y_out)
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.title("Outer-loop step response with inner loop closed")
plt.grid(True)
plt.show()

# Frequency response to inspect bandwidth separation
w = np.logspace(-2, 2, 300)
mag_i, phase_i, w_i = ct.bode(Ci * Gp, w, Plot=False)
mag_o, phase_o, w_o = ct.bode(L_outer, w, Plot=False)

# Rough estimate of crossover frequencies
def crossover(w_vals, mag_vals):
    idx = np.argmin(np.abs(mag_vals - 1.0))
    return w_vals[idx]

w_ci = crossover(w_i, mag_i)
w_co = crossover(w_o, mag_o)
print("Inner-loop crossover ~", w_ci, "rad/s")
print("Outer-loop crossover ~", w_co, "rad/s")
