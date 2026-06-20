import numpy as np
import pinocchio as pin

# 1) Load a robot model (URDF path and reference frame adapted to your setup)
model = pin.buildModelFromUrdf("path/to/your_robot.urdf")
data = model.createData()
nq = model.nq
nv = model.nv

# 2) Multi-sine parameterization for each joint
M = 3                      # number of harmonics per joint
T = 5.0                    # experiment duration [s]
dt = 0.002                 # sampling period [s]
t = np.arange(0.0, T, dt)
N = t.size

# Example parameters (to be later optimized)
q0   = np.zeros(nq)
A    = 0.4 * np.ones((nq, M))      # amplitudes
W    = np.linspace(1.0, 5.0, M)    # base frequencies [rad/s]
PHI0 = np.linspace(0.0, np.pi/2, M)

def multi_sine_joint_traj(t, q0, A_row, W, PHI0):
    """
    Single-joint multi-sine q(t), qdot(t), qddot(t).
    A_row: shape (M,)
    """
    q = np.full_like(t, q0, dtype=float)
    qd = np.zeros_like(t, dtype=float)
    qdd = np.zeros_like(t, dtype=float)
    for m in range(len(W)):
        w = W[m]
        a = A_row[m]
        ph = PHI0[m]
        s = np.sin(w * t + ph)
        c = np.cos(w * t + ph)
        q += a * s
        qd += a * w * c
        qdd += -a * (w ** 2) * s
    return q, qd, qdd

# 3) Build joint-space trajectories (q, qdot, qddot) for all joints
q_traj   = np.zeros((N, nq))
qd_traj  = np.zeros((N, nq))
qdd_traj = np.zeros((N, nq))

for i in range(nq):
    q_i, qd_i, qdd_i = multi_sine_joint_traj(t, q0[i], A[i, :], W, PHI0)
    q_traj[:, i]   = q_i
    qd_traj[:, i]  = qd_i
    qdd_traj[:, i] = qdd_i

# 4) For each sample, compute the dynamic regressor using Pinocchio
Y_list = []
for k in range(N):
    qk = q_traj[k, :]
    qdk = qd_traj[k, :]
    qddk = qdd_traj[k, :]
    # Ensure correct vector types (column vectors)
    qk = np.asarray(qk).reshape(-1, 1)
    qdk = np.asarray(qdk).reshape(-1, 1)
    qddk = np.asarray(qddk).reshape(-1, 1)

    # Compute regressor: Y_k so that tau = Y_k * theta
    Yk = pin.computeJointTorqueRegressor(model, data, qk, qdk, qddk)
    # Yk has shape (nv, p)
    Y_list.append(Yk)

# Stack into Phi
Phi = np.vstack(Y_list)   # shape (N*nv, p)

# 5) Information matrix and condition number
F = Phi.T @ Phi
eigs = np.linalg.eigvalsh(F)
lambda_min = np.min(eigs)
lambda_max = np.max(eigs)
cond = lambda_max / lambda_min
print("lambda_min =", lambda_min)
print("cond(F)    =", cond)
      
