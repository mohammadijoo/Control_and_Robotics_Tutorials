import numpy as np

def reflected_inertia_single(J_m, J_L, n, J_g=0.0):
    """
    Compute equivalent inertia seen at joint for a geared single-DOF system.
    J_m : motor inertia
    J_L : link inertia at joint
    n   : gear ratio (theta_m = n * q)
    J_g : gear lumped inertia on motor side
    """
    J_eq_q = n**2 * (J_m + J_g) + J_L
    J_eq_m = (J_m + J_g) + J_L / (n**2)
    return J_eq_q, J_eq_m

def joint_dynamics(t, x, params):
    """
    State x = [q, qdot].
    Dynamics: J_eq * qdd + b_eq * qdot + k_load * q = n * eta * tau_m + tau_ext
    Here we use a simple linear spring load and constant motor torque.
    """
    q, qdot = x
    J_eq = params["J_eq"]
    b_eq = params["b_eq"]
    k_load = params["k_load"]
    n = params["n"]
    eta = params["eta"]
    tau_m = params["tau_m"]
    tau_ext = params["tau_ext"]

    # Equation: J_eq * qdd = n*eta*tau_m + tau_ext - b_eq*qdot - k_load*q
    qddot = (n * eta * tau_m + tau_ext - b_eq * qdot - k_load * q) / J_eq
    return np.array([qdot, qddot])

def simulate_forward_euler(f, x0, t_grid, params):
    x = np.zeros((len(t_grid), len(x0)))
    x[0] = x0
    for k in range(len(t_grid) - 1):
        dt = t_grid[k+1] - t_grid[k]
        xdot = f(t_grid[k], x[k], params)
        x[k+1] = x[k] + dt * xdot
    return x

if __name__ == "__main__":
    # Physical parameters
    J_m = 0.002     # kg*m^2
    J_L = 0.05      # kg*m^2
    J_g = 0.001     # gear inertia
    b_m = 0.001     # motor viscous friction
    b_q = 0.02      # joint viscous friction
    k_load = 5.0    # Nm/rad (e.g. elastic load)
    tau_ext = 0.0   # external torque
    tau_m = 1.0     # step motor torque (Nm)
    eta = 0.9       # efficiency

    # Compare two gear ratios
    t_grid = np.linspace(0.0, 1.0, 200)
    x0 = np.array([0.0, 0.0])

    for n in [10.0, 100.0]:
        J_eq_q, J_eq_m = reflected_inertia_single(J_m, J_L, n, J_g)
        b_eq = n**2 * b_m + b_q

        params = {
            "J_eq": J_eq_q,
            "b_eq": b_eq,
            "k_load": k_load,
            "n": n,
            "eta": eta,
            "tau_m": tau_m,
            "tau_ext": tau_ext,
        }

        x_traj = simulate_forward_euler(joint_dynamics, x0, t_grid, params)
        q_traj = x_traj[:, 0]
        print(f"Gear ratio n={n}: final joint angle = {q_traj[-1]:.3f} rad")
      
