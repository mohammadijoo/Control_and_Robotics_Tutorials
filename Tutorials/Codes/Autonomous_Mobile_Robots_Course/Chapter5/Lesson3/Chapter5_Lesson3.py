# Chapter5_Lesson3.py
# Autonomous Mobile Robots (Control Engineering) — Chapter 5, Lesson 3
# Topic: Drift Sources and Bias Accumulation
#
# This script simulates 2D dead-reckoning for a differential-drive robot and
# illustrates how (i) systematic biases (wheel radius mismatch, wheelbase error,
# gyro bias) and (ii) stochastic errors (encoder quantization, noise) accumulate.
#
# Dependencies: numpy, matplotlib

import numpy as np
import matplotlib.pyplot as plt

def wrap_pi(a):
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

def simulate(
    T=60.0, dt=0.01,
    # True geometry
    r_true=0.05, b_true=0.30, ticks_per_rev=2048,
    # Estimator (possibly wrong) geometry
    r_hat=0.05*1.005, b_hat=0.30*0.995,
    # Systematic wheel mismatch (true radii differ)
    eps_r_L=+0.002, eps_r_R=-0.002,
    # Encoder quantization + noise
    encoder_tick_noise_std=0.2,   # in ticks (Gaussian), optional
    # Gyro
    gyro_bias=0.005,             # rad/s constant bias
    gyro_noise_std=0.002,        # rad/s white noise
    seed=7
):
    rng = np.random.default_rng(seed)
    N = int(np.floor(T/dt))
    t = np.arange(N)*dt

    # Motion command: piecewise curvature so drift becomes visible
    v_cmd = 0.6*np.ones(N)                           # m/s
    w_cmd = 0.10*np.sin(2*np.pi*t/20.0)              # rad/s

    # True wheel radii (mismatch)
    rL_true = r_true*(1.0 + eps_r_L)
    rR_true = r_true*(1.0 + eps_r_R)

    # State arrays: ground truth and estimates
    x = np.zeros(N); y = np.zeros(N); th = np.zeros(N)
    xw = np.zeros(N); yw = np.zeros(N); thw = np.zeros(N)   # wheel-only estimate
    xg = np.zeros(N); yg = np.zeros(N); thg = np.zeros(N)   # gyro-heading + wheel-distance

    # Gyro integrated heading (starts aligned)
    th_gyro = 0.0

    # Encoder quantization step (radians per tick at wheel)
    rad_per_tick = 2*np.pi / ticks_per_rev

    for k in range(1, N):
        v = v_cmd[k-1]; w = w_cmd[k-1]

        # True wheel angular velocities
        wR = (v + 0.5*b_true*w)/rR_true
        wL = (v - 0.5*b_true*w)/rL_true

        # True wheel angle increments
        dphiR_true = wR*dt
        dphiL_true = wL*dt

        # Convert to ticks (quantize) + optional tick noise
        ticksR = dphiR_true / rad_per_tick
        ticksL = dphiL_true / rad_per_tick

        ticksR_meas = np.round(ticksR) + rng.normal(0.0, encoder_tick_noise_std)
        ticksL_meas = np.round(ticksL) + rng.normal(0.0, encoder_tick_noise_std)

        # Reconstruct measured wheel angle increments from ticks (quantized)
        dphiR_meas = ticksR_meas * rad_per_tick
        dphiL_meas = ticksL_meas * rad_per_tick

        # True kinematics update (ground truth)
        dSR_true = rR_true*dphiR_true
        dSL_true = rL_true*dphiL_true
        dS_true  = 0.5*(dSR_true + dSL_true)
        dTh_true = (dSR_true - dSL_true)/b_true

        th[k] = wrap_pi(th[k-1] + dTh_true)
        x[k]  = x[k-1] + dS_true*np.cos(th[k-1] + 0.5*dTh_true)
        y[k]  = y[k-1] + dS_true*np.sin(th[k-1] + 0.5*dTh_true)

        # Wheel-only odometry estimate (uses r_hat, b_hat)
        dSR_hat = r_hat*dphiR_meas
        dSL_hat = r_hat*dphiL_meas
        dS_hat  = 0.5*(dSR_hat + dSL_hat)
        dTh_hat = (dSR_hat - dSL_hat)/b_hat

        thw[k] = wrap_pi(thw[k-1] + dTh_hat)
        xw[k]  = xw[k-1] + dS_hat*np.cos(thw[k-1] + 0.5*dTh_hat)
        yw[k]  = yw[k-1] + dS_hat*np.sin(thw[k-1] + 0.5*dTh_hat)

        # Gyro measurement and integrated heading
        w_meas = w + gyro_bias + rng.normal(0.0, gyro_noise_std)
        th_gyro = wrap_pi(th_gyro + w_meas*dt)

        # Gyro-heading + wheel-distance: use th_gyro for direction, wheel distance for magnitude
        thg[k] = th_gyro
        xg[k]  = xg[k-1] + dS_hat*np.cos(thg[k-1])  # use previous heading
        yg[k]  = yg[k-1] + dS_hat*np.sin(thg[k-1])

    out = {
        "t": t,
        "truth": np.vstack([x,y,th]).T,
        "wheel": np.vstack([xw,yw,thw]).T,
        "gyro":  np.vstack([xg,yg,thg]).T,
        "v_cmd": v_cmd,
        "w_cmd": w_cmd
    }
    return out

def main():
    data = simulate()

    t = data["t"]
    xt, yt, tht = data["truth"].T
    xw, yw, thw = data["wheel"].T
    xg, yg, thg = data["gyro"].T

    # Errors
    e_w = np.sqrt((xw-xt)**2 + (yw-yt)**2)
    e_g = np.sqrt((xg-xt)**2 + (yg-yt)**2)

    print("Final position error (wheel-only) [m]:", float(e_w[-1]))
    print("Final position error (gyro-heading) [m]:", float(e_g[-1]))
    print("Final heading error (wheel-only) [deg]:", float(np.degrees(wrap_pi(thw[-1]-tht[-1]))))
    print("Final heading error (gyro-heading) [deg]:", float(np.degrees(wrap_pi(thg[-1]-tht[-1]))))

    # Plots
    plt.figure()
    plt.plot(xt, yt, label="truth")
    plt.plot(xw, yw, label="wheel-only")
    plt.plot(xg, yg, label="gyro-heading + wheel-distance")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Dead-reckoning drift under bias and noise")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t, e_w, label="wheel-only position error")
    plt.plot(t, e_g, label="gyro-heading position error")
    plt.xlabel("time [s]")
    plt.ylabel("||position error|| [m]")
    plt.title("Error accumulation over time")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t, np.degrees(wrap_pi(thw-tht)), label="wheel-only heading error")
    plt.plot(t, np.degrees(wrap_pi(thg-tht)), label="gyro-heading heading error")
    plt.xlabel("time [s]")
    plt.ylabel("heading error [deg]")
    plt.title("Heading drift (bias accumulation)")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
