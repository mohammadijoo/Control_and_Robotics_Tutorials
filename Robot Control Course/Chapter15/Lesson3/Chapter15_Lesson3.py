
import numpy as np
from numpy.linalg import norm
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

# Physical parameters (unknown to the controller, but used for data generation)
m_true = 1.0
l_true = 0.5
g = 9.81
b_true = 0.2  # viscous friction coefficient

def true_dynamics(q, dq, tau):
    """
    1-DOF rotary joint:
        I * ddq + b_true * dq + m_true * g * l_true * sin(q) = tau
    """
    I = m_true * l_true**2
    ddq = (tau - b_true * dq - m_true * g * l_true * np.sin(q)) / I
    return ddq

# Nominal model used in controller (misses friction)
m_nom = 0.9       # slightly wrong mass
l_nom = 0.45
def nominal_inverse_dynamics(q, dq, ddq):
    I0 = m_nom * l_nom**2
    tau_nom = I0 * ddq + m_nom * g * l_nom * np.sin(q)
    return tau_nom

# Generate training data under some excitation trajectory
def generate_training_data(T=5.0, dt=0.01):
    t = np.arange(0.0, T, dt)
    N = t.size
    q = 0.5 * np.sin(2.0 * t)
    dq = np.gradient(q, dt)
    ddq_des = np.gradient(dq, dt)

    X = []
    y = []

    q_curr = 0.0
    dq_curr = 0.0
    for k in range(N):
        # Nominal control: inverse dynamics for desired ddq plus PD
        e = q[k] - q_curr
        de = dq[k] - dq_curr
        kp = 50.0
        kd = 10.0
        v = ddq_des[k] + kd * de + kp * e
        tau_nom = nominal_inverse_dynamics(q_curr, dq_curr, v)

        # Apply to true plant and simulate one step
        ddq_true = true_dynamics(q_curr, dq_curr, tau_nom)
        dq_curr = dq_curr + ddq_true * dt
        q_curr = q_curr + dq_curr * dt

        # Compute residual torque that would make nominal model exact
        tau_inv_nom = nominal_inverse_dynamics(q_curr, dq_curr, ddq_true)
        d = tau_nom - tau_inv_nom  # torque residual

        x_k = np.array([q_curr, dq_curr, ddq_des[k]])
        X.append(x_k)
        y.append(d)

    return np.asarray(X), np.asarray(y)

X_train, y_train = generate_training_data()

# Fit GP on residual torque
kernel = 1.0 * RBF(length_scale=np.ones(3)) + WhiteKernel(noise_level=1e-4)
gp = GaussianProcessRegressor(kernel=kernel, alpha=0.0, normalize_y=True)
gp.fit(X_train, y_train)

print("Trained GP kernel:", gp.kernel_)

# Control loop with GP compensation
def control_with_gp(q0, dq0, qd_fun, dqd_fun, ddqd_fun, T=3.0, dt=0.002):
    t_grid = np.arange(0.0, T, dt)
    q = q0
    dq = dq0
    traj = {"t": [], "q": [], "qd": [], "dq": [], "tau": []}

    kp = 80.0
    kd = 20.0

    for t in t_grid:
        qd = qd_fun(t)
        dqd = dqd_fun(t)
        ddqd = ddqd_fun(t)

        e = qd - q
        de = dqd - dq

        v = ddqd + kd * de + kp * e
        tau_nom = nominal_inverse_dynamics(q, dq, v)

        x = np.array([[q, dq, ddqd]])
        d_hat, sigma = gp.predict(x, return_std=True)
        tau = tau_nom + d_hat[0]

        ddq_true = true_dynamics(q, dq, tau)
        dq = dq + ddq_true * dt
        q = q + dq * dt

        traj["t"].append(t)
        traj["q"].append(q)
        traj["qd"].append(qd)
        traj["dq"].append(dq)
        traj["tau"].append(tau)

    for key in traj:
        traj[key] = np.asarray(traj[key])
    return traj

# Example: track a slow sinusoid
qd_fun = lambda t: 0.3 * np.sin(1.0 * t)
dqd_fun = lambda t: 0.3 * np.cos(1.0 * t)
ddqd_fun = lambda t: -0.3 * np.sin(1.0 * t)

traj_gp = control_with_gp(0.0, 0.0, qd_fun, dqd_fun, ddqd_fun)

print("Final position error:", traj_gp["qd"][-1] - traj_gp["q"][-1])
