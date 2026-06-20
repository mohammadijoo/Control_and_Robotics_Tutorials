# Chapter14_Lesson5.py
# Limit Cycles, Multiple Equilibria, and Basic Bifurcation Notions
# Python implementation: Van der Pol oscillator + 1D bifurcation normal forms
import numpy as np
import matplotlib.pyplot as plt

def rk4_step(f, t, x, h, params):
    k1 = f(t, x, params)
    k2 = f(t + 0.5*h, x + 0.5*h*k1, params)
    k3 = f(t + 0.5*h, x + 0.5*h*k2, params)
    k4 = f(t + h, x + h*k3, params)
    return x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def vdp_rhs(t, state, params):
    mu = params["mu"]
    x, y = state
    dx = y
    dy = mu*(1.0 - x*x)*y - x
    return np.array([dx, dy], dtype=float)

def simulate_vdp(mu=1.0, x0=(2.0, 0.0), T=80.0, h=0.01):
    n = int(T/h)
    t = np.linspace(0.0, T, n+1)
    x = np.zeros((n+1, 2))
    x[0] = np.array(x0, dtype=float)
    params = {"mu": mu}
    for k in range(n):
        x[k+1] = rk4_step(vdp_rhs, t[k], x[k], h, params)
    return t, x

def estimate_period_from_crossings(t, traj):
    # Upward crossings of y=0 (x-axis): use y sign change from negative to positive
    y = traj[:, 1]
    x = traj[:, 0]
    crossings = []
    for k in range(len(t)-1):
        if y[k] < 0.0 and y[k+1] >= 0.0:
            # Linear interpolation
            alpha = -y[k] / (y[k+1] - y[k] + 1e-14)
            tc = t[k] + alpha*(t[k+1] - t[k])
            xc = x[k] + alpha*(x[k+1] - x[k])
            if xc > 0:  # consistent Poincare section branch
                crossings.append(tc)
    if len(crossings) < 3:
        return np.nan
    # Discard transient crossing intervals
    periods = np.diff(crossings)
    if len(periods) >= 4:
        periods = periods[-4:]
    return float(np.mean(periods))

def classify_pitchfork(r, x):
    # f(x)=r x - x^3 ; stability from f'(x)=r - 3 x^2
    lam = r - 3.0*x*x
    if lam < 0:
        return "stable"
    elif lam > 0:
        return "unstable"
    return "nonhyperbolic"

def classify_saddlenode(r, x):
    # f(x)=r - x^2 ; stability from f'(x)=-2x
    lam = -2.0*x
    if lam < 0:
        return "stable"
    elif lam > 0:
        return "unstable"
    return "nonhyperbolic"

def main():
    # 1) Van der Pol simulation for several mu values
    mu_list = [0.2, 1.0, 3.0]
    results = []
    fig1 = plt.figure(figsize=(7, 5))
    for mu in mu_list:
        t, z = simulate_vdp(mu=mu, x0=(2.0, 0.1), T=80.0, h=0.01)
        # discard transient for plotting
        z_ss = z[int(0.5*len(z)):, :]
        plt.plot(z_ss[:,0], z_ss[:,1], label=f"mu={mu}")
        period = estimate_period_from_crossings(t[int(0.4*len(t)):], z[int(0.4*len(z)):, :])
        amp = float(np.max(np.abs(z_ss[:,0])))
        results.append((mu, amp, period))
    plt.xlabel("x")
    plt.ylabel("x_dot")
    plt.title("Van der Pol Limit Cycles (phase plane)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # 2) 1D bifurcation normal forms (equilibrium branches)
    r_vals = np.linspace(-2.0, 2.0, 401)
    pf_stable_r, pf_stable_x = [], []
    pf_unstable_r, pf_unstable_x = [], []
    sn_stable_r, sn_stable_x = [], []
    sn_unstable_r, sn_unstable_x = [], []

    # Pitchfork: x*=0 and ±sqrt(r) for r>=0
    for r in r_vals:
        for xeq in [0.0]:
            c = classify_pitchfork(r, xeq)
            (pf_stable_r if c == "stable" else pf_unstable_r).append(r)
            (pf_stable_x if c == "stable" else pf_unstable_x).append(xeq)
        if r >= 0:
            for xeq in [np.sqrt(r), -np.sqrt(r)]:
                c = classify_pitchfork(r, xeq)
                (pf_stable_r if c == "stable" else pf_unstable_r).append(r)
                (pf_stable_x if c == "stable" else pf_unstable_x).append(xeq)

    # Saddle-node: x*=±sqrt(r) for r>=0
    for r in r_vals:
        if r >= 0:
            for xeq in [np.sqrt(r), -np.sqrt(r)]:
                c = classify_saddlenode(r, xeq)
                (sn_stable_r if c == "stable" else sn_unstable_r).append(r)
                (sn_stable_x if c == "stable" else sn_unstable_x).append(xeq)

    fig2 = plt.figure(figsize=(7, 5))
    plt.plot(pf_stable_r, pf_stable_x, linewidth=2, label="Pitchfork stable")
    plt.plot(pf_unstable_r, pf_unstable_x, "--", linewidth=2, label="Pitchfork unstable")
    plt.plot(sn_stable_r, sn_stable_x, linewidth=2, label="Saddle-node stable")
    plt.plot(sn_unstable_r, sn_unstable_x, "--", linewidth=2, label="Saddle-node unstable")
    plt.xlabel("parameter r")
    plt.ylabel("equilibrium x*")
    plt.title("Basic Bifurcation Branches")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # 3) Print numerical summary
    print("Van der Pol asymptotic limit-cycle estimates")
    print("mu\tamplitude(|x|)\tperiod")
    for mu, amp, period in results:
        print(f"{mu:.2f}\t{amp:.4f}\t\t{period:.4f}")

    plt.show()

if __name__ == "__main__":
    main()
