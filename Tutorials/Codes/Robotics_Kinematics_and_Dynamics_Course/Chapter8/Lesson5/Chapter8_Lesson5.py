import numpy as np

# 2R Jacobian
def jacobian_2r(q, l1=1.0, l2=1.0):
    q1, q2 = q
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)

    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ], dtype=float)
    return J

def manipulability(J):
    JJt = J @ J.T
    return float(np.sqrt(np.linalg.det(JJt)))

def joint_limit_penalty(q, q_min, q_max):
    q = np.asarray(q, dtype=float)
    q_min = np.asarray(q_min, dtype=float)
    q_max = np.asarray(q_max, dtype=float)
    phi = 0.0
    for qi, ql, qu in zip(q, q_min, q_max):
        phi += 1.0 / (qi - ql)**2 + 1.0 / (qu - qi)**2
    return phi

def fk_2r(q, l1=1.0, l2=1.0):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y], dtype=float)

def workspace_penalty(q, l1=1.0, l2=1.0, eps_ws=1e-3):
    x = fk_2r(q, l1, l2)
    r = np.linalg.norm(x)
    r_min = abs(l1 - l2)
    r_max = l1 + l2
    d_ws = min(r - r_min, r_max - r)
    return 1.0 / (d_ws**2 + eps_ws)

def avoidance_cost(q,
                   l1=1.0, l2=1.0,
                   q_min=(-np.pi, -np.pi),
                   q_max=( np.pi,  np.pi),
                   eps=1e-3):
    J = jacobian_2r(q, l1, l2)
    w = manipulability(J)
    phi_sing = 1.0 / (w**2 + eps)
    phi_joint = joint_limit_penalty(q, q_min, q_max)
    phi_ws = workspace_penalty(q, l1, l2)
    return phi_sing + phi_joint + phi_ws

def finite_diff_grad(f, q, h=1e-4):
    q = np.asarray(q, dtype=float)
    g = np.zeros_like(q)
    for i in range(len(q)):
        dq = np.zeros_like(q)
        dq[i] = h
        g[i] = (f(q + dq) - f(q - dq)) / (2.0 * h)
    return g

def gradient_step(q, step=1e-2):
    # One offline gradient step to reduce avoidance cost
    cost_fun = lambda x: avoidance_cost(x)
    g = finite_diff_grad(cost_fun, np.array(q, dtype=float))
    return np.array(q, dtype=float) - step * g

if __name__ == "__main__":
    q = np.array([0.0, 0.5])
    for k in range(20):
        c = avoidance_cost(q)
        print(f"Iter {k}: q = {q}, cost = {c:.6f}")
        q = gradient_step(q, step=5e-2)
      
