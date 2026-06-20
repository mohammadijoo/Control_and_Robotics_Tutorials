import numpy as np
from numpy.linalg import lstsq

# "Real" data generation (unknown to identification)
np.random.seed(0)
m_true = 2.0
b_true = 0.8
Ts = 0.01
N = 2000

u = 2.0*np.sin(0.5*np.arange(N)*Ts)  # known input
v = np.zeros(N)
for k in range(N-1):
    vdot = (u[k] - b_true*v[k]) / m_true
    v[k+1] = v[k] + Ts*vdot
v_meas = v + 0.02*np.random.randn(N)  # noisy measurement

# Identification model: v_{k+1} - v_k = Ts*(u_k/m - (b/m) v_k)
# Fix m to known nominal (from datasheet), estimate b by LS.
m_nom = 2.0
y = (v_meas[1:] - v_meas[:-1]) / Ts - u[:-1]/m_nom
Phi = (-v_meas[:-1] / m_nom).reshape(-1,1)
b_hat, _, _, _ = lstsq(Phi, y, rcond=None)
b_hat = float(b_hat)

print("Estimated b:", b_hat, "True b:", b_true)

# Validation on fresh input
Nv = 800
u_val = 1.5*np.cos(0.7*np.arange(Nv)*Ts)
v_real = np.zeros(Nv)
v_sim  = np.zeros(Nv)

for k in range(Nv-1):
    # real (unknown to sim)
    v_real[k+1] = v_real[k] + Ts*(u_val[k] - b_true*v_real[k])/m_true
    # sim with estimated friction
    v_sim[k+1]  = v_sim[k]  + Ts*(u_val[k] - b_hat*v_sim[k])/m_nom

e = v_real - v_sim
rmse = np.sqrt(np.mean(e**2))
maxerr = np.max(np.abs(e))
print("Validation RMSE:", rmse, "Max error:", maxerr)

# t-test against tolerance epsilon
epsilon = 0.05
z = e**2
zbar = np.mean(z)
s = np.std(z, ddof=1)
t_stat = (zbar - epsilon**2) / (s/np.sqrt(Nv))
print("t statistic:", t_stat)
      
