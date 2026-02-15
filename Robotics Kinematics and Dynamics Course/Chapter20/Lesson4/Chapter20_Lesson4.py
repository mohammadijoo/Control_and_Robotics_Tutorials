import numpy as np

def f(x, tau, I, b=0.1, k=1.0):
    q, dq = x
    ddq = (tau - b * dq - k * q) / I
    return np.array([dq, ddq])

def step_euler(x, tau, I, dt):
    return x + dt * f(x, tau, I)

def rollout(x0, taus, I, dt):
    xs = [x0]
    for tau in taus:
        xs.append(step_euler(xs[-1], tau, I, dt))
    return np.array(xs)  # shape: (N+1, 2)

def loss(xs, q_ref):
    # xs[:,0] contains q
    return 0.5 * np.sum((xs[:, 0] - q_ref)**2)

def grad_I_adjoint(x0, taus, I, dt, q_ref):
    xs = rollout(x0, taus, I, dt)
    N = len(taus)
    # adjoint lambdas: lambda_k in R^2
    lambdas = np.zeros_like(xs)
    # terminal condition
    lambdas[N, 0] = xs[N, 0] - q_ref[N]  # d l_N / d q_N
    # precompute A_k and dPhi/dI for explicit Euler
    A = np.zeros((N, 2, 2))
    dPhi_dI = np.zeros((N, 2))
    for k in range(N):
        q, dq = xs[k]
        tau = taus[k]
        # partial derivatives of f wrt x
        # df/dq = [0, -(k/I)]
        # df/ddq = [1, -(b/I)]
        df_dq = np.array([0.0, -1.0 * k / I])
        df_ddq = np.array([1.0, -1.0 * 0.1 / I])
        Jx = np.column_stack([df_dq, df_ddq])  # 2x2
        A[k] = np.eye(2) + dt * Jx
        # df/dI
        ddq = (tau - 0.1 * dq - k * q) / I
        dddq_dI = -(tau - 0.1 * dq - k * q) / (I**2)
        df_dI = np.array([0.0, dddq_dI])
        dPhi_dI[k] = dt * df_dI
    # backward adjoint recursion
    for k in range(N - 1, -1, -1):
        # gradient of stage loss wrt x_k
        dL_dxk = np.zeros(2)
        dL_dxk[0] = xs[k, 0] - q_ref[k]
        lambdas[k] = dL_dxk + A[k].T @ lambdas[k + 1]
    # parameter gradient
    dL_dI = 0.0
    for k in range(N):
        dL_dI += lambdas[k + 1] @ dPhi_dI[k]
    return dL_dI

# Example usage
dt = 0.01
N = 100
taus = np.ones(N) * 1.0
t_grid = np.arange(N + 1) * dt
q_ref = 0.5 * np.sin(2.0 * np.pi * t_grid)
x0 = np.array([0.0, 0.0])
I = 0.5

xs = rollout(x0, taus, I, dt)
print("Loss:", loss(xs, q_ref))
print("dL/dI (adjoint):", grad_I_adjoint(x0, taus, I, dt, q_ref))
      
