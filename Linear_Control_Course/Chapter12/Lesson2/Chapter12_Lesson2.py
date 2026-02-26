import numpy as np
import control as ct  # python-control library
# (Optional) robotics-specific libraries:
# import roboticstoolbox as rtb

# First-order plant approximating a robot joint actuator:
#   G(s) = K / (T s + 1)
K = 20.0   # static gain
T = 0.1    # time constant (s)
G = ct.tf([K], [T, 1.0])

# Desired specifications
omega_c_des = 40.0          # rad/s (desired bandwidth)
pm_des_deg = 60.0           # desired phase margin (deg)
pm_des = pm_des_deg * np.pi / 180.0

def pid_tf(Kp, Ki, Kd):
    """Ideal PID transfer function C(s) = Kp + Ki/s + Kd s."""
    s = ct.tf([1.0, 0.0], [1.0])
    return Kp + Ki / s + Kd * s

def loop_margins(Kp, Ki, Kd):
    """Return crossover frequency and phase margin of the loop."""
    C = pid_tf(Kp, Ki, Kd)
    L = C * G
    gm, pm, wcg, wcp = ct.margin(L)  # gain margin, phase margin, crossover freqs
    return wcg, wcp, pm

# Simple search over Ti, Td around the plant time constant
Ti_candidates = [0.05, 0.1, 0.2]   # integral time candidates
Td_candidates = [0.02, 0.05, 0.08] # derivative time candidates

best = None
for Ti in Ti_candidates:
    for Td in Td_candidates:
        # compute Ki, Kd from Kp, Ti, Td relations
        # Step 1: compute Kp from magnitude equation at omega_c_des
        w = omega_c_des
        # plant magnitude |G(jw)|
        magG = K / np.sqrt(1.0 + (w * T)**2)
        # PID magnitude factor sqrt(1 + (w Td - 1/(w Ti))^2)
        factor = np.sqrt(1.0 + (w * Td - 1.0 / (w * Ti))**2)
        Kp = np.sqrt(1.0 + (w * T)**2) / (K * factor)
        Ki = Kp / Ti
        Kd = Kp * Td

        wcg, wcp, pm = loop_margins(Kp, Ki, Kd)

        # score: closeness to desired phase margin and bandwidth
        if wcp is None:  # no crossover found
            continue
        score = abs(wcp - omega_c_des) / omega_c_des + abs(pm - pm_des)
        if best is None or score < best["score"]:
            best = dict(Ti=Ti, Td=Td, Kp=Kp, Ki=Ki, Kd=Kd,
                        wcp=wcp, pm=pm, score=score)

print("Best design:", best)

# Controller suitable for deployment in a robot joint torque or position loop:
C_pid = pid_tf(best["Kp"], best["Ki"], best["Kd"])
L = C_pid * G
ct.bode(L)  # visualize loop shape
