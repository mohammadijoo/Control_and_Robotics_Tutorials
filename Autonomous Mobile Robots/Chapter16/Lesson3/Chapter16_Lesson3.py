# Chapter16_Lesson3.py
# Socially Aware Navigation (sampling-based local velocity selection)
# Dependencies: numpy, matplotlib

import numpy as np
import matplotlib.pyplot as plt

def rot2(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s],[s, c]])

def anisotropic_gaussian_cost(x, humans, sigma_front=1.2, sigma_side=0.6, sigma_back=0.8):
    """
    Social discomfort field around each human i:
      C_i(x) = exp(-0.5 * (x-p_i)^T Sigma_i^{-1} (x-p_i))
    with anisotropy aligned with the human heading (estimated from velocity).
    """
    cost = 0.0
    for h in humans:
        p = h["p"]
        v = h["v"]
        spd = np.linalg.norm(v)
        th = np.arctan2(v[1], v[0]) if spd > 1e-6 else 0.0
        R = rot2(th)

        # front/back asymmetry: choose sigma_x depending on whether x is in front of the human
        r = x - p
        r_h = R.T @ r
        sx = sigma_front if r_h[0] >= 0.0 else sigma_back
        Sy = np.diag([sx**2, sigma_side**2])

        q = r_h.T @ np.linalg.inv(Sy) @ r_h
        cost += np.exp(-0.5 * q)
    return cost

def collision_within_tau(v_robot, p_robot, humans, R=0.55, tau=3.0):
    """
    Check if robot velocity v_robot would collide with any human within horizon tau,
    using closest-approach analysis on relative motion (constant velocity model).
    """
    for h in humans:
        p_h, v_h = h["p"], h["v"]
        p = p_h - p_robot
        v_rel = v_robot - v_h
        vv = float(v_rel @ v_rel)
        if vv < 1e-10:
            if np.linalg.norm(p) < R:
                return True
            continue
        t_star = - float(p @ v_rel) / vv
        t_star = max(0.0, min(tau, t_star))
        d_min = np.linalg.norm(p + t_star * (-v_rel))
        if d_min < R:
            return True
    return False

def sample_velocities(v_pref, v_max=1.2, n_speed=9, n_angle=21, spread=np.deg2rad(80)):
    spref = np.linalg.norm(v_pref)
    th0 = np.arctan2(v_pref[1], v_pref[0]) if spref > 1e-8 else 0.0
    speeds = np.linspace(0.0, v_max, n_speed)
    angles = np.linspace(-spread, spread, n_angle)
    V = []
    for s in speeds:
        for a in angles:
            th = th0 + a
            V.append(np.array([s*np.cos(th), s*np.sin(th)]))
    return V

def choose_velocity(p_robot, v_pref, humans,
                    v_max=1.2, w_track=1.0, w_social=1.5, w_clear=2.0,
                    R=0.55, tau=3.0):
    best_v = np.zeros(2)
    best_J = float("inf")
    eps = 1e-3

    for v in sample_velocities(v_pref, v_max=v_max):
        if collision_within_tau(v, p_robot, humans, R=R, tau=tau):
            continue

        dt_eval = 0.6
        p_next = p_robot + dt_eval * v

        C = anisotropic_gaussian_cost(p_next, humans)
        clear = 0.0
        for h in humans:
            d = np.linalg.norm(p_next - h["p"])
            clear += 1.0/(d + eps)

        J = w_track*np.sum((v - v_pref)**2) + w_social*C + w_clear*clear
        if J < best_J:
            best_J = J
            best_v = v

    return best_v, best_J

def simulate(seed=2):
    np.random.seed(seed)
    dt = 0.1
    T = 26.0
    steps = int(T/dt)

    p = np.array([0.0, 0.0])
    goal = np.array([10.0, 0.0])
    v_max = 1.2

    humans = [
        {"p": np.array([4.5,  1.2]), "v": np.array([ 0.35, -0.05])},
        {"p": np.array([4.0, -1.4]), "v": np.array([ 0.35,  0.08])},
        {"p": np.array([7.0,  0.3]), "v": np.array([-0.25,  0.02])},
    ]

    traj = [p.copy()]
    traj_h = [[h["p"].copy()] for h in humans]

    for _ in range(steps):
        for i, h in enumerate(humans):
            h["p"] = h["p"] + dt*h["v"]
            traj_h[i].append(h["p"].copy())

        e = goal - p
        dist = np.linalg.norm(e)
        if dist < 0.15:
            break
        v_pref = (e / (dist + 1e-9)) * min(v_max, 0.8*dist)

        v_cmd, _ = choose_velocity(p, v_pref, humans, v_max=v_max, tau=3.0)
        p = p + dt*v_cmd
        traj.append(p.copy())

    traj = np.array(traj)
    plt.figure(figsize=(8,5))
    plt.plot(traj[:,0], traj[:,1], label="robot")
    for i, th in enumerate(traj_h):
        th = np.array(th)
        plt.plot(th[:,0], th[:,1], "--", label=f"human {i+1}")
        plt.scatter(th[0,0], th[0,1], s=30)
    plt.scatter(goal[0], goal[1], marker="*", s=120)
    plt.axis("equal"); plt.grid(True); plt.legend()
    plt.title("Socially aware navigation: sampling-based local velocity selection")
    plt.show()

if __name__ == "__main__":
    simulate()
