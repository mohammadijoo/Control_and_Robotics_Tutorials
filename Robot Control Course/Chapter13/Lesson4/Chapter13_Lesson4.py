
import numpy as np
import matplotlib.pyplot as plt

def safe_input(x, u_des, u_min, u_max, kappa):
    """
    Safety filter for dot{x} = u with CBF h(x) = x and input box [u_min, u_max].

    Parameters
    ----------
    x : float
        Current state.
    u_des : float
        Nominal (possibly unsafe) input.
    u_min, u_max : float
        Actuator limits with u_min <= u_max.
    kappa : float
        CBF gain (alpha(s) = kappa * s).
    """
    # Lower bound from CBF + actuator limit
    u_low = max(u_min, -kappa * x)
    # Upper bound from actuator limit
    u_high = u_max
    # Projection of u_des onto [u_low, u_high]
    u_safe = np.minimum(u_high, np.maximum(u_low, u_des))
    return float(u_safe)

def simulate_safety_filter(x0, u_min, u_max, kappa, k_p, T, dt):
    """
    Closed-loop simulation with nominal controller u_des = -k_p * (x - x_ref),
    where x_ref = 1. Safety filter keeps x(t) >= 0.

    Parameters
    ----------
    x0 : float
        Initial state, should satisfy x0 >= 0.
    u_min, u_max : float
        Input limits.
    kappa : float
        CBF gain.
    k_p : float
        Proportional gain of nominal controller.
    T : float
        Simulation horizon.
    dt : float
        Time step.
    """
    x = x0
    t_values = [0.0]
    x_values = [x0]
    u_nom_values = []
    u_safe_values = []

    x_ref = 1.0
    N = int(T / dt)

    for k in range(N):
        t = (k + 1) * dt
        # Nominal proportional controller (ignores safety)
        u_des = -k_p * (x - x_ref)
        # Safety filter with input limits
        u_safe = safe_input(x, u_des, u_min, u_max, kappa)

        # Integrate 1D dynamics: x_{k+1} = x_k + dt * u_safe
        x = x + dt * u_safe

        t_values.append(t)
        x_values.append(x)
        u_nom_values.append(u_des)
        u_safe_values.append(u_safe)

    return np.array(t_values), np.array(x_values), np.array(u_nom_values), np.array(u_safe_values)

if __name__ == "__main__":
    # Parameters
    x0 = 0.1         # start close to the constraint
    u_min = -2.0
    u_max =  2.0
    kappa = 5.0
    k_p = 5.0
    T = 5.0
    dt = 0.001

    t, x, u_nom, u_safe = simulate_safety_filter(
        x0=x0, u_min=u_min, u_max=u_max,
        kappa=kappa, k_p=k_p, T=T, dt=dt
    )

    # Plot results
    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(t, x)
    axs[0].axhline(0.0, linestyle="--")
    axs[0].set_ylabel("x(t)")
    axs[0].set_title("State with Safety Filter and Input Limits")

    axs[1].plot(t[1:], u_nom, label="u_des")
    axs[1].plot(t[1:], u_safe, label="u_safe", linestyle="--")
    axs[1].axhline(u_min, linestyle=":", label="u_min")
    axs[1].axhline(u_max, linestyle=":", label="u_max")
    axs[1].set_xlabel("time")
    axs[1].set_ylabel("input")
    axs[1].legend()

    plt.tight_layout()
    plt.show()
