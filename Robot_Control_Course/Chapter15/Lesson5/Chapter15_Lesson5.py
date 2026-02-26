
import numpy as np

# Optional: scikit-learn for Gaussian Processes
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

# True parameters
m_true = 1.2
b_true = 0.3
g_true = 9.81

# Nominal parameters (slightly wrong)
m_nom = 1.0
b_nom = 0.1
g_nom = 9.81

# Unknown disturbance term (unmodeled dynamics)
def disturbance(q, qd):
    # Smooth but nonlinear "extra" friction and flex effect
    return 0.4 * np.sin(2.0 * q) + 0.05 * qd**3

def true_dynamics(q, qd, tau):
    d = disturbance(q, qd)
    qdd = (tau - b_true * qd - g_true * np.sin(q) - d) / m_true
    return qdd, d

def nominal_torque(q, qd, qd_des, qdd_des, Kp, Kd):
    e = q - qd_des
    edot = qd - (qd_des)  # qd_des is dot(q_d); passed in explicitly
    v = qdd_des - Kd * edot - Kp * e
    tau_nom = m_nom * v + b_nom * qd + g_nom * np.sin(q)
    return tau_nom

# Desired trajectory (moderately rich)
def desired_trajectory(t):
    # q_d(t), qd_d(t), qdd_d(t)
    A = 0.7
    w = 0.7
    qd = A * np.sin(w * t)
    qd_dot = A * w * np.cos(w * t)
    qd_ddot = -A * w**2 * np.sin(w * t)
    return qd, qd_dot, qd_ddot

# Simulation parameters
dt = 0.002
T = 12.0
N = int(T / dt)

Kp = 25.0
Kd = 10.0

def rollout(with_residual=False, gpr=None, r_max=2.0, collect_data=False):
    q = 0.0
    qd = 0.0
    qs = []
    qds = []
    qd_des_list = []
    qdd_des_list = []
    taus = []
    Z = []
    Y = []

    for k in range(N):
        t = k * dt
        qd_des, qd_dot_des, qdd_des = desired_trajectory(t)
        tau_nom = nominal_torque(q, qd, qd_des, qdd_des, Kp, Kd)

        z = np.array([q, qd, qd_des, qdd_des], dtype=float)

        if with_residual and gpr is not None:
            # Predict residual and saturate
            r_hat = gpr.predict(z.reshape(1, -1))[0]
            r_hat = float(np.clip(r_hat, -r_max, r_max))
        else:
            r_hat = 0.0

        tau = tau_nom + r_hat

        # True dynamics integration (semi-implicit Euler)
        qdd, d_true = true_dynamics(q, qd, tau)
        qd = qd + dt * qdd
        q = q + dt * qd

        # Store logs
        qs.append(q)
        qds.append(qd)
        qd_des_list.append(qd_des)
        qdd_des_list.append(qdd_des)
        taus.append(tau)

        if collect_data:
            y = tau - tau_nom   # target residual
            Z.append(z)
            Y.append(y)

    result = {
        "q": np.array(qs),
        "qd": np.array(qds),
        "q_d": np.array(qd_des_list),
        "qd_d": np.gradient(np.array(qd_des_list), dt),
        "qdd_d": np.array(qdd_des_list),
        "tau": np.array(taus),
    }
    if collect_data:
        result["Z"] = np.array(Z)
        result["Y"] = np.array(Y)
    return result

# 1) Collect data under nominal controller
nominal_run = rollout(with_residual=False, collect_data=True)
Z_train = nominal_run["Z"]
Y_train = nominal_run["Y"]

# 2) Fit a scalar-output GP residual model
kernel = 1.0 * RBF(length_scale=0.5) + WhiteKernel(noise_level=1e-4)
gpr = GaussianProcessRegressor(kernel=kernel, alpha=0.0, normalize_y=True)
gpr.fit(Z_train, Y_train)

print("Fitted GP kernel:", gpr.kernel_)

# 3) Evaluate tracking with and without residual
baseline = rollout(with_residual=False, collect_data=False)
augmented = rollout(with_residual=True, gpr=gpr, r_max=2.0, collect_data=False)

# 4) Compute RMS tracking error
def rms(x):
    return np.sqrt(np.mean(x**2))

err_base = baseline["q"] - baseline["q_d"]
err_aug = augmented["q"] - augmented["q_d"]

print("RMS error baseline :", rms(err_base))
print("RMS error augmented:", rms(err_aug))
