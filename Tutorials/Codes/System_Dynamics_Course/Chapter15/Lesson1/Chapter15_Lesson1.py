# Chapter15_Lesson1.py
# Numerical simulation of an IVP using Euler and Improved Euler (Heun) methods
# Example IVP: y' = -2 y + sin(t), y(0) = 1

import numpy as np
import matplotlib.pyplot as plt


def f(t, y):
    return -2.0 * y + np.sin(t)


def exact_solution(t):
    # y(t) = (2 sin t - cos t)/5 + (6/5) e^{-2 t}
    return (2.0 * np.sin(t) - np.cos(t)) / 5.0 + (6.0 / 5.0) * np.exp(-2.0 * t)


def euler(fhandle, t0, tf, y0, h):
    n = int(round((tf - t0) / h))
    t = np.linspace(t0, tf, n + 1)
    y = np.zeros(n + 1)
    y[0] = y0
    for k in range(n):
        y[k + 1] = y[k] + h * fhandle(t[k], y[k])
    return t, y


def improved_euler(fhandle, t0, tf, y0, h):
    """Heun / explicit trapezoid method."""
    n = int(round((tf - t0) / h))
    t = np.linspace(t0, tf, n + 1)
    y = np.zeros(n + 1)
    y[0] = y0
    for k in range(n):
        slope1 = fhandle(t[k], y[k])
        y_pred = y[k] + h * slope1
        slope2 = fhandle(t[k] + h, y_pred)
        y[k + 1] = y[k] + 0.5 * h * (slope1 + slope2)
    return t, y


def max_abs_error(t, y_num):
    return np.max(np.abs(y_num - exact_solution(t)))


def estimate_order(err_h, err_h2):
    return np.log(err_h / err_h2) / np.log(2.0)


def main():
    t0, tf, y0 = 0.0, 5.0, 1.0
    hs = [0.2, 0.1, 0.05, 0.025]

    print("Step-size study (max-norm error on [0, 5])")
    print("h        Euler error      Heun error")
    e_errors = []
    h_errors = []
    for h in hs:
        t_e, y_e = euler(f, t0, tf, y0, h)
        t_h, y_h = improved_euler(f, t0, tf, y0, h)
        ee = max_abs_error(t_e, y_e)
        he = max_abs_error(t_h, y_h)
        e_errors.append(ee)
        h_errors.append(he)
        print(f"{h:<8.3f} {ee:<14.6e} {he:<14.6e}")

    print("\nEstimated orders (using successive halving)")
    for i in range(len(hs) - 1):
        p_e = estimate_order(e_errors[i], e_errors[i + 1])
        p_h = estimate_order(h_errors[i], h_errors[i + 1])
        print(f"h={hs[i]:.3f} -> {hs[i+1]:.3f}: Euler p≈{p_e:.3f}, Heun p≈{p_h:.3f}")

    # Plot one representative step size
    h_plot = 0.1
    t_e, y_e = euler(f, t0, tf, y0, h_plot)
    t_h, y_h = improved_euler(f, t0, tf, y0, h_plot)
    t_dense = np.linspace(t0, tf, 800)
    y_dense = exact_solution(t_dense)

    plt.figure(figsize=(9, 5))
    plt.plot(t_dense, y_dense, label="Exact")
    plt.plot(t_e, y_e, "o-", markersize=3, label="Euler (h=0.1)")
    plt.plot(t_h, y_h, "s-", markersize=3, label="Improved Euler (h=0.1)")
    plt.xlabel("t")
    plt.ylabel("y(t)")
    plt.title("IVP solution: Euler vs Improved Euler")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Save CSV for cross-language comparison
    out_data = np.column_stack([t_h, y_h, exact_solution(t_h), np.abs(y_h - exact_solution(t_h))])
    np.savetxt("Chapter15_Lesson1_output.csv", out_data, delimiter=",",
               header="t,heun,exact,abs_error", comments="")


if __name__ == "__main__":
    main()
