import numpy as np
import matplotlib.pyplot as plt
import control  # python-control: pip install control
# Optional robotics toolbox (Peter Corke)
# from roboticstoolbox import DHRobot, RevoluteDH

# Parameter ranges for robot joint PD control
J_vals = np.linspace(0.5, 2.0, 20)      # inertia range
B_vals = np.linspace(0.2, 1.0, 20)      # viscous friction range
Kp_vals = np.linspace(5.0, 40.0, 10)    # proportional gain range
Kd_vals = np.linspace(0.1, 5.0, 10)     # derivative gain range

zeta_min = 0.5
Mp_max = 0.15
ts_max = 2.0

def closed_loop_metrics(J, B, Kp, Kd):
    # Plant G(s) = 1 / (J s^2 + B s)
    numG = [1.0]
    denG = [J, B, 0.0]
    G = control.TransferFunction(numG, denG)

    # PD controller C(s) = Kp + Kd s
    numC = [Kd, Kp]
    denC = [1.0]
    C = control.TransferFunction(numC, denC)

    T = control.feedback(C*G, 1)  # unity feedback

    # Step response and performance metrics
    t, y = control.step_response(T)
    info = control.step_info(T)

    Mp = info["Overshoot"] / 100.0  # convert percent to fraction
    ts = info["SettlingTime"]

    # Approximate damping by dominant poles
    poles = control.pole(T)
    # take pole with largest real part (closest to imaginary axis)
    p_dom = poles[np.argmax(np.real(poles))]
    wn = abs(p_dom)
    zeta = -np.real(p_dom) / wn if wn > 0 else 1.0

    return zeta, Mp, ts, poles

stable_count = 0
robust_count = 0
total_count = 0

for J in J_vals:
    for B in B_vals:
        for Kp in Kp_vals:
            for Kd in Kd_vals:
                total_count += 1
                zeta, Mp, ts, poles = closed_loop_metrics(J, B, Kp, Kd)

                if np.all(np.real(poles) < 0.0):
                    stable_count += 1
                    if (zeta >= zeta_min) and (Mp <= Mp_max) and (ts <= ts_max):
                        robust_count += 1

print("Stable fraction:", stable_count / total_count)
print("Robust-performance fraction:", robust_count / total_count)

# Example visualization: map of spec satisfaction over (Kp, Kd) for nominal (J,B)
J_nom, B_nom = 1.0, 0.5
sat_map = np.zeros((len(Kp_vals), len(Kd_vals)), dtype=int)

for i, Kp in enumerate(Kp_vals):
    for j, Kd in enumerate(Kd_vals):
        zeta, Mp, ts, poles = closed_loop_metrics(J_nom, B_nom, Kp, Kd)
        if np.all(np.real(poles) < 0.0) and (zeta >= zeta_min) and (Mp <= Mp_max) and (ts <= ts_max):
            sat_map[i, j] = 1

Kd_grid, Kp_grid = np.meshgrid(Kd_vals, Kp_vals)
plt.figure()
plt.contourf(Kd_grid, Kp_grid, sat_map, levels=[-0.5, 0.5, 1.5])
plt.xlabel("Kd")
plt.ylabel("Kp")
plt.title("Spec-satisfying region (1 = ok, 0 = fail)")
plt.colorbar()
plt.show()
