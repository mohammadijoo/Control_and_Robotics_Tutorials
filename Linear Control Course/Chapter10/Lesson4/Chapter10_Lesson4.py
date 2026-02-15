import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Robot joint parameters (example values)
J = 0.01   # inertia
b = 0.1    # viscous damping
K_m = 0.5  # motor constant

# Plant: G(s) = K_m / (J s^2 + b s) = K_m / (s (J s + b))
num_G = [K_m]
den_G = [J, b, 0.0]
G = ctrl.TransferFunction(num_G, den_G)

# Desired specs
zeta_min = 0.6
Ts_max = 1.0
sigma_min = -4.0 / Ts_max  # vertical line

# Choose tentative lead compensator parameters (manual initial guess)
z_c = 10.0   # zero at s = -z_c
p_c = 50.0   # pole at s = -p_c, placed further left for approximate phase lead
C0 = ctrl.TransferFunction([1.0, z_c], [1.0, p_c])

L = G * C0

# Plot root locus and overlay lines of constant zeta and sigma
fig, ax = plt.subplots()
rlocus_data = ctrl.root_locus(L, Plot=True, ax=ax, grid=False)

# Overlay damping ratio rays and vertical line using sgrid
zeta_list = [zeta_min]
wn_list = np.linspace(0.1, 50.0, 200)
for zeta in zeta_list:
    # ray: sigma = -zeta * wn, omega = wn * sqrt(1 - zeta^2)
    sigma = -zeta * wn_list
    omega = wn_list * np.sqrt(1.0 - zeta**2)
    ax.plot(sigma, omega, linestyle="--")
    ax.plot(sigma, -omega, linestyle="--")

ax.axvline(sigma_min, linestyle="--")  # settling time vertical line

ax.set_xlabel("Real axis")
ax.set_ylabel("Imag axis")
ax.set_title("Root locus with initial lead compensator")

plt.show()

# Search for gain K such that dominant pole lies inside admissible region
def dominant_poles_for_gain(K):
    closed_loop = ctrl.feedback(K * L, 1)
    poles = ctrl.pole(closed_loop)
    # dominant poles: those with largest real part (closest to imaginary axis)
    real_parts = np.real(poles)
    idx = np.argsort(real_parts)[-2:]  # two poles with largest real part
    return poles[idx]

def specs_satisfied(pole):
    sigma = np.real(pole)
    omega = np.imag(pole)
    wn = np.sqrt(sigma**2 + omega**2)
    if wn == 0.0:
        return False
    zeta = -sigma / wn
    Ts = 4.0 / (-sigma)
    return (zeta >= zeta_min) and (Ts <= Ts_max)

K_candidates = np.logspace(-1, 3, 200)
good_pairs = []
for K in K_candidates:
    poles = dominant_poles_for_gain(K)
    if any(specs_satisfied(p) for p in poles):
        good_pairs.append((K, poles))

print("Number of gains meeting specs with current compensator:", len(good_pairs))
if good_pairs:
    K_star, poles_star = good_pairs[0]
    print("Example acceptable gain:", K_star)
    print("Dominant poles:", poles_star)
