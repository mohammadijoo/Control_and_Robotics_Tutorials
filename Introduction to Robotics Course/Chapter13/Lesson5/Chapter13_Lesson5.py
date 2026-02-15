import numpy as np

# ---------- True (physical) parameters ----------
J_true, b_true, Kt_true = 0.02, 0.1, 0.05
Ke_true, R_true, L_true = 0.05, 2.0, 0.5

# ---------- Twin parameters (initial guesses) ----------
J_hat, b_hat, Kt_hat = 0.03, 0.2, 0.04
Ke_hat, R_hat, L_hat = Ke_true, R_true, L_true  # assume known for simplicity

dt = 0.01
T = 5.0
N = int(T/dt)

# Input voltage
t = np.arange(N)*dt
V = 6.0 * (t > 0.5)  # step after 0.5s

# Storage
x_true = np.zeros((N,2))  # [omega, i]
x_hat  = np.zeros((N,2))
y_meas = np.zeros(N)

# Observer gain (chosen stable)
L_obs = np.array([30.0, 5.0])  # [l1, l2]^T

def f_motor(x, V, J, b, Kt, Ke, R, L):
    omega, i = x
    domega = (-b/J)*omega + (Kt/J)*i
    di     = (-Ke/L)*omega - (R/L)*i + (1.0/L)*V
    return np.array([domega, di])

# ---------- Simulate physical system and twin ----------
for k in range(N-1):
    # physical model
    x_true[k+1] = x_true[k] + dt*f_motor(x_true[k], V[k],
                                        J_true, b_true, Kt_true, Ke_true, R_true, L_true)
    y_meas[k] = x_true[k,0] + np.random.normal(0, 0.02)  # measure omega

    # twin predictor
    x_hat[k+1] = x_hat[k] + dt*f_motor(x_hat[k], V[k],
                                      J_hat, b_hat, Kt_hat, Ke_hat, R_hat, L_hat)

    # correction (Luenberger)
    residual = y_meas[k] - x_hat[k,0]
    x_hat[k+1] += dt*L_obs*residual

# ---------- Least-squares update for theta1=b/J and theta2=Kt/J ----------
omega = y_meas
i_hat = x_hat[:,1]
domega = np.diff(omega)/dt
Phi = np.column_stack((-omega[:-1], i_hat[:-1]))
theta_ls = np.linalg.inv(Phi.T@Phi) @ (Phi.T@domega)

theta1_hat, theta2_hat = theta_ls
print("Estimated theta1=b/J:", theta1_hat)
print("Estimated theta2=Kt/J:", theta2_hat)

# Update twin parameters using J_hat (keeping J_hat fixed here)
b_hat  = theta1_hat * J_hat
Kt_hat = theta2_hat * J_hat
print("Updated b_hat, Kt_hat:", b_hat, Kt_hat)
      
