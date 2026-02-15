import numpy as np
import matplotlib.pyplot as plt
from control import tf, feedback, nichols_plot, magphase

# Plant G(s) = 1 / (s (s + 1))
s = tf([1, 0], [1])
G = 1 / (s * (s + 1))

# Frequency grid
w = np.logspace(-2, 2, 500)

# Design requirement (informal):
# - target crossover around wc ~ 1 rad/s
# - approximate resonant peak Mp <= 2 (about 6 dB)
Mp_target = 2.0

# --- Step 1: proportional controller ---
def closed_loop_peak_mag(K):
    """Compute peak |T(jw)| for proportional gain K."""
    L = K * G
    T = feedback(L, 1)
    mag, phase, _ = magphase(T(w), deg=True)
    return mag.max(), mag, phase

Kp_guess = 5.0
Mp, magT, phaseT = closed_loop_peak_mag(Kp_guess)
print(f"Proportional K = {Kp_guess:.2f}, peak |T| = {Mp:.3f}")

# Nichols plot of loop L(s) = K G(s)
L_prop = Kp_guess * G
plt.figure()
nichols_plot(L_prop, w)
plt.title("Nichols plot for proportional controller")
plt.grid(True)

# --- Step 2: simple first-order compensator K(s) = Kc (tau_z s + 1)/(tau_p s + 1) ---
def make_compensator(Kc, tau_z, tau_p):
    return Kc * tf([tau_z, 1], [tau_p, 1])

def design_compensator(Kc_init=5.0, tau_z=0.5, tau_p=0.1):
    Kc = Kc_init
    K = make_compensator(Kc, tau_z, tau_p)
    L = K * G
    T = feedback(L, 1)
    mag, phase, _ = magphase(T(w), deg=True)
    print(f"Kc={Kc:.2f}, tau_z={tau_z:.3f}, tau_p={tau_p:.3f}, peak |T| = {mag.max():.3f}")
    return K, L, T, mag, phase

Kc0 = 4.0
K_dyn, L_dyn, T_dyn, mag_dyn, phase_dyn = design_compensator(Kc0, tau_z=0.5, tau_p=0.05)

plt.figure()
nichols_plot(L_dyn, w)
plt.title("Nichols plot with first-order dynamic compensator")
plt.grid(True)

plt.show()
