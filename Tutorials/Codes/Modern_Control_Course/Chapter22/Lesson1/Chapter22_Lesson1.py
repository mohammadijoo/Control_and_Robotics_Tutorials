# Chapter22_Lesson1.py
# Structure of state-feedback law: u = -K x + r
# Requires: numpy; optional matplotlib for plotting

import numpy as np

def rk4_step(f, t, x, h):
    k1 = f(t, x)
    k2 = f(t + 0.5*h, x + 0.5*h*k1)
    k3 = f(t + 0.5*h, x + 0.5*h*k2)
    k4 = f(t + h, x + h*k3)
    return x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def controllability_rank(A, B):
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    Ctrb = np.hstack(blocks)
    return np.linalg.matrix_rank(Ctrb), Ctrb

def simulate_closed_loop(A, B, C, K, r_value=1.0, x0=None, tf=8.0, h=0.01):
    n = A.shape[0]
    if x0 is None:
        x0 = np.zeros(n)
    x = x0.astype(float).copy()
    Acl = A - B @ K
    r_vec = np.array([[r_value]], dtype=float)

    def f(t, x_vec):
        x_col = x_vec.reshape(-1, 1)
        dx = Acl @ x_col + B @ r_vec
        return dx.ravel()

    ts = np.arange(0.0, tf + h, h)
    xs = np.zeros((len(ts), n))
    us = np.zeros(len(ts))
    ys = np.zeros(len(ts))

    for i, t in enumerate(ts):
        xs[i, :] = x
        x_col = x.reshape(-1, 1)
        u = float((-K @ x_col + r_vec)[0, 0])
        y = float((C @ x_col)[0, 0])
        us[i] = u
        ys[i] = y
        if i + 1 < len(ts):
            x = rk4_step(f, t, x, h)

    return ts, xs, us, ys, Acl

def main():
    # Mass-spring-damper example:
    # x1 = position, x2 = velocity, u = force
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])

    # State-feedback gain. This lesson studies the structure;
    # later lessons will derive systematic pole-placement formulas.
    K = np.array([[4.0, 2.6]])

    rank, Ctrb = controllability_rank(A, B)
    print("Controllability matrix:\n", Ctrb)
    print("Rank:", rank)

    ts, xs, us, ys, Acl = simulate_closed_loop(A, B, C, K, r_value=1.0, x0=np.array([0.2, 0.0]))

    print("A - B K =\n", Acl)
    print("Closed-loop eigenvalues:", np.linalg.eigvals(Acl))

    # Constant-r equilibrium: x_bar = -(A-BK)^(-1) B r
    r_value = 1.0
    x_bar = -np.linalg.solve(Acl, B * r_value)
    y_bar = C @ x_bar
    print("Equilibrium x_bar:", x_bar.ravel())
    print("Equilibrium y_bar:", float(y_bar[0, 0]))

    # Optional plot
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(ts, ys, label="y = Cx")
        plt.plot(ts, us, label="u = -Kx + r")
        plt.xlabel("time [s]")
        plt.grid(True)
        plt.legend()
        plt.title("Chapter22 Lesson1: State-feedback structure")
        plt.show()
    except Exception as exc:
        print("Plot skipped:", exc)

if __name__ == "__main__":
    main()
