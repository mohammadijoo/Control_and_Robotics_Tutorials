import numpy as np
import matplotlib.pyplot as plt
from control import TransferFunction, step_response

# Nominal joint parameters (e.g., small robot arm link)
J0 = 0.01   # kg m^2
b0 = 0.05   # N m s/rad
k0 = 2.0    # N m/rad

# Relative uncertainty bounds (+/- 20%)
rel_unc = 0.2
N = 200  # number of Monte Carlo samples

def sample_params():
    def sample_one(p0):
        # uniform in [p0*(1-rel_unc), p0*(1+rel_unc)]
        return p0 * (1.0 + rel_unc * (2.0*np.random.rand() - 1.0))
    J = sample_one(J0)
    b = sample_one(b0)
    k = sample_one(k0)
    return J, b, k

poles_real = []
poles_imag = []

for _ in range(N):
    J, b, k = sample_params()
    # Second-order denominator: J s^2 + b s + k
    den = [J, b, k]
    num = [1.0]
    G = TransferFunction(num, den)
    lam = np.roots(den)  # open-loop poles of the joint dynamics
    for p in lam:
        poles_real.append(np.real(p))
        poles_imag.append(np.imag(p))

# Plot pole cloud
plt.figure()
plt.scatter(poles_real, poles_imag, marker='x')
plt.axvline(x=0.0, linestyle='--')  # imaginary axis
plt.xlabel("Re(s)")
plt.ylabel("Im(s)")
plt.title("Joint pole locations under parametric uncertainty")

# Example step response for nominal model
G_nom = TransferFunction([1.0], [J0, b0, k0])
t, y = step_response(G_nom)
plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("theta(t) [rad]")
plt.title("Nominal joint step response")
plt.show()
