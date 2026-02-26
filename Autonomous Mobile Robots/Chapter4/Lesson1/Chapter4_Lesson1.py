"""
Chapter4_Lesson1.py
Autonomous Mobile Robots - Chapter 4 (Mobile Robot Dynamics Applied)
Lesson 1: When Dynamics Matter for AMR

This script compares:
  (1) a kinematic unicycle model driven directly by (v_cmd, w_cmd)
  (2) a simple dynamic model where (v, w) evolve due to bounded wheel torques

Author: Course materials generator
"""

import numpy as np
import matplotlib.pyplot as plt

def rk4_step(f, t, x, dt, u):
    k1 = f(t, x, u)
    k2 = f(t + 0.5*dt, x + 0.5*dt*k1, u)
    k3 = f(t + 0.5*dt, x + 0.5*dt*k2, u)
    k4 = f(t + dt, x + dt*k3, u)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def sat(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

# -------------------------
# Models
# -------------------------
def kin_unicycle_rhs(t, x, u):
    # x = [px, py, th]
    px, py, th = x
    v_cmd, w_cmd = u
    return np.array([
        v_cmd*np.cos(th),
        v_cmd*np.sin(th),
        w_cmd
    ], dtype=float)

def dyn_diffdrive_rhs_factory(params):
    """
    Simple planar dynamics:
      m * v_dot = (tau_R + tau_L)/r_w - b_v * v
      I * w_dot = (b/(2 r_w))*(tau_R - tau_L) - b_w * w

    Pose kinematics:
      px_dot = v cos(th), py_dot = v sin(th), th_dot = w
    """
    m   = params["m"]
    Iz  = params["Iz"]
    rw  = params["rw"]
    b   = params["b"]
    bv  = params["bv"]
    bw  = params["bw"]
    tau_max = params["tau_max"]

    kv = params["kv"]
    kw = params["kw"]

    def rhs(t, x, u):
        # x = [px, py, th, v, w]
        px, py, th, v, w = x
        v_ref, w_ref = u

        # Outer-loop (very simplified): map velocity errors to wheel torques
        tau_sum  = kv*(v_ref - v)          # affects linear acceleration
        tau_diff = kw*(w_ref - w)          # affects yaw acceleration

        tau_R = 0.5*(tau_sum + tau_diff)
        tau_L = 0.5*(tau_sum - tau_diff)

        tau_R = float(sat(tau_R, -tau_max, tau_max))
        tau_L = float(sat(tau_L, -tau_max, tau_max))

        Fx = (tau_R + tau_L)/rw
        Mz = (b/(2.0*rw))*(tau_R - tau_L)

        v_dot = (Fx - bv*v)/m
        w_dot = (Mz - bw*w)/Iz

        return np.array([
            v*np.cos(th),
            v*np.sin(th),
            w,
            v_dot,
            w_dot
        ], dtype=float)

    return rhs

# -------------------------
# Reference command profile
# -------------------------
def cmd_profile(t):
    # Smoothly varying commands to highlight acceleration limits
    # Segment 1: accelerate and turn
    # Segment 2: sharper turn and higher speed demand
    if t <= 5.0:
        v_cmd = 0.2 + 0.18*t          # ramps to about 1.1
        w_cmd = 0.6                   # constant turn
    else:
        v_cmd = 1.1
        w_cmd = 1.5                   # sharper turn demand
    return np.array([v_cmd, w_cmd], dtype=float)

def main():
    dt = 0.002
    T  = 10.0
    ts = np.arange(0.0, T + dt, dt)

    # Kinematic initial state
    xk = np.array([0.0, 0.0, 0.0], dtype=float)

    # Dynamic initial state
    params = dict(
        m=30.0,           # kg
        Iz=1.2,           # kg m^2
        rw=0.10,          # m
        b=0.50,           # m (wheel track)
        bv=6.0,           # N s/m (lumped rolling drag)
        bw=0.25,          # N m s/rad (lumped yaw drag)
        tau_max=3.0,      # N m (per wheel)

        kv=8.0,           # torque gain for v tracking
        kw=1.2            # torque gain for w tracking
    )
    xd = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    rhs_dyn = dyn_diffdrive_rhs_factory(params)

    Xk = np.zeros((ts.size, 3))
    Xd = np.zeros((ts.size, 5))
    U  = np.zeros((ts.size, 2))

    for i, t in enumerate(ts):
        u = cmd_profile(t)
        U[i, :] = u

        Xk[i, :] = xk
        Xd[i, :] = xd

        xk = rk4_step(kin_unicycle_rhs, t, xk, dt, u)
        xk[2] = wrap_pi(xk[2])

        xd = rk4_step(rhs_dyn, t, xd, dt, u)
        xd[2] = wrap_pi(xd[2])

    # Plot trajectories and commanded vs achieved velocities
    fig1 = plt.figure()
    plt.plot(Xk[:, 0], Xk[:, 1], label="Kinematic (ideal v,w)")
    plt.plot(Xd[:, 0], Xd[:, 1], label="Dynamic (torque-limited)")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory: kinematic vs dynamic")
    plt.legend()
    plt.grid(True)

    fig2 = plt.figure()
    plt.plot(ts, U[:, 0], label="v_cmd")
    plt.plot(ts, Xd[:, 3], label="v (dynamic)")
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.title("Linear speed tracking (torque limited)")
    plt.legend()
    plt.grid(True)

    fig3 = plt.figure()
    plt.plot(ts, U[:, 1], label="w_cmd")
    plt.plot(ts, Xd[:, 4], label="w (dynamic)")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.title("Yaw-rate tracking (torque limited)")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
