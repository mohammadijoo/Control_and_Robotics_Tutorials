import numpy as np

def planar2r_fk(q, l1, l2):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def planar2r_jacobian(q, l1, l2):
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)
    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ])
    return J

def central_diff(x, h):
    x = np.asarray(x)
    dx = np.zeros_like(x)
    # one-sided at boundaries
    dx[0]  = (x[1] - x[0]) / h
    dx[-1] = (x[-1] - x[-2]) / h
    for k in range(1, len(x) - 1):
        dx[k] = (x[k + 1] - x[k - 1]) / (2.0 * h)
    return dx

# trajectory definition
l1, l2 = 1.0, 0.7
omega = 1.5
T = 5.0
N = 501
t = np.linspace(0.0, T, N)
h = t[1] - t[0]

q = np.zeros((N, 2))
for k in range(N):
    q[k, 0] = np.sin(omega * t[k])
    q[k, 1] = np.cos(omega * t[k])

# joint velocities: numerical (componentwise)
dq_num = np.zeros_like(q)
dq_num[:, 0] = central_diff(q[:, 0], h)
dq_num[:, 1] = central_diff(q[:, 1], h)

# joint velocities: analytic
dq_ana = np.zeros_like(q)
dq_ana[:, 0] = omega * np.cos(omega * t)
dq_ana[:, 1] = -omega * np.sin(omega * t)

# task-space trajectories and velocities
x = np.zeros((N, 2))
xd_num = np.zeros((N, 2))
xd_model = np.zeros((N, 2))
for k in range(N):
    x[k] = planar2r_fk(q[k], l1, l2)

# numerical task-space velocity
xd_num[:, 0] = central_diff(x[:, 0], h)
xd_num[:, 1] = central_diff(x[:, 1], h)

# model-based task-space velocity (using analytic dq_ana)
for k in range(N):
    J = planar2r_jacobian(q[k], l1, l2)
    xd_model[k] = J @ dq_ana[k]

# simple RMS error comparison
rms_err_num = np.sqrt(np.mean((xd_num - xd_model) ** 2))
print("RMS difference between numerical and model-based xdot:", rms_err_num)

# Note: with noise added to q, xd_num will deviate much more than xd_model.
      
