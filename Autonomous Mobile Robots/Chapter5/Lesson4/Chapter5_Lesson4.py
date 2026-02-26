"""
Chapter5_Lesson4.py
Autonomous Mobile Robots — Chapter 5 Lesson 4: Practical Odometry Filtering

This script demonstrates practical, deterministic filtering for odometry streams:
- Robust outlier suppression (Hampel filter)
- First-order low-pass filtering (IIR)
- Complementary heading fusion: wheel-based heading (low-frequency) + gyro integration (high-frequency)
- Simple kinematic integration of planar pose (x, y, theta)

Dependencies: numpy, scipy (optional, only for comparison), matplotlib
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt


def wrap_to_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a


@dataclass
class FirstOrderLowPass:
    """y[k] = alpha*y[k-1] + (1-alpha)*x[k] with 0<alpha<1."""
    alpha: float
    y: float = 0.0
    initialized: bool = False

    @staticmethod
    def from_cutoff(fc_hz: float, dt: float) -> "FirstOrderLowPass":
        # Exact discretization of tau*dy/dt + y = x with tau = 1/(2*pi*fc)
        tau = 1.0 / (2.0 * math.pi * fc_hz)
        alpha = math.exp(-dt / tau)
        return FirstOrderLowPass(alpha=alpha)

    def step(self, x: float) -> float:
        if not self.initialized:
            self.y = x
            self.initialized = True
            return self.y
        self.y = self.alpha * self.y + (1.0 - self.alpha) * x
        return self.y


@dataclass
class HampelFilter:
    """
    Hampel filter for outlier suppression:
      x[k] replaced by median if |x[k]-median| > n_sigmas * 1.4826*MAD
    """
    window: int = 11
    n_sigmas: float = 3.0

    def apply(self, x: np.ndarray) -> np.ndarray:
        assert self.window % 2 == 1, "window must be odd"
        k = self.window // 2
        y = x.copy()
        for i in range(k, len(x) - k):
            w = x[i - k : i + k + 1]
            med = np.median(w)
            mad = np.median(np.abs(w - med)) + 1e-12
            sigma = 1.4826 * mad
            if abs(x[i] - med) > self.n_sigmas * sigma:
                y[i] = med
        return y


@dataclass
class ComplementaryHeadingFilter:
    """
    Discrete complementary fusion for heading:
        theta_hat[k] = alpha*(theta_hat[k-1] + dt*omega_g[k]) + (1-alpha)*theta_w[k]
    where theta_w is wheel-based heading (low-freq stable), omega_g is gyro yaw rate (high-freq responsive).
    """
    alpha: float
    theta_hat: float = 0.0
    initialized: bool = False

    @staticmethod
    def from_cutoff(fc_hz: float, dt: float) -> "ComplementaryHeadingFilter":
        tau = 1.0 / (2.0 * math.pi * fc_hz)
        alpha = math.exp(-dt / tau)
        return ComplementaryHeadingFilter(alpha=alpha)

    def step(self, omega_g: float, theta_w: float, dt: float) -> float:
        if not self.initialized:
            self.theta_hat = theta_w
            self.initialized = True
            return self.theta_hat
        pred = wrap_to_pi(self.theta_hat + dt * omega_g)
        self.theta_hat = wrap_to_pi(self.alpha * pred + (1.0 - self.alpha) * theta_w)
        return self.theta_hat


@dataclass
class OdometryFilterConfig:
    dt: float = 0.01
    v_fc_hz: float = 5.0
    w_fc_hz: float = 8.0
    theta_fc_hz: float = 0.7
    v_max: float = 2.0
    w_max: float = 3.0
    slip_gate_rad_s: float = 1.2  # gate residual between gyro and wheel yaw rate


class OdometryFilter:
    def __init__(self, cfg: OdometryFilterConfig):
        self.cfg = cfg
        self.v_lp = FirstOrderLowPass.from_cutoff(cfg.v_fc_hz, cfg.dt)
        self.w_lp = FirstOrderLowPass.from_cutoff(cfg.w_fc_hz, cfg.dt)
        self.theta_cf = ComplementaryHeadingFilter.from_cutoff(cfg.theta_fc_hz, cfg.dt)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self, v_w: float, w_w: float, w_g: float, theta_w: float) -> Tuple[float, float, float]:
        # Physical sanity clamp
        v_w = float(np.clip(v_w, -self.cfg.v_max, self.cfg.v_max))
        w_w = float(np.clip(w_w, -self.cfg.w_max, self.cfg.w_max))
        w_g = float(np.clip(w_g, -self.cfg.w_max, self.cfg.w_max))

        # Slip gating: if wheel yaw rate disagrees strongly with gyro, downweight wheel yaw (use gyro path more)
        resid = abs(w_g - w_w)
        if resid > self.cfg.slip_gate_rad_s:
            # increase alpha temporarily (more reliance on gyro integration)
            alpha_eff = min(0.995, self.theta_cf.alpha + 0.15)
        else:
            alpha_eff = self.theta_cf.alpha

        # Low-pass filter velocities (reduces quantization and high-frequency jitter)
        v_f = self.v_lp.step(v_w)
        w_f = self.w_lp.step(w_w)

        # Complementary heading fusion (use gyro yaw rate w_g, wheel heading theta_w)
        # Use alpha_eff for this step
        pred = wrap_to_pi(self.theta + self.cfg.dt * w_g)
        theta_hat = wrap_to_pi(alpha_eff * pred + (1.0 - alpha_eff) * theta_w)

        # Integrate pose using filtered linear velocity and fused heading
        self.theta = theta_hat
        self.x += self.cfg.dt * v_f * math.cos(self.theta)
        self.y += self.cfg.dt * v_f * math.sin(self.theta)

        return self.x, self.y, self.theta


def simulate_truth(T: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate ground-truth v(t), w(t), and theta(t) for a planar robot."""
    N = int(T / dt)
    t = np.arange(N) * dt

    # piecewise commands
    v = np.zeros(N)
    w = np.zeros(N)
    for k in range(N):
        if 1.0 <= t[k] < 6.0:
            v[k] = 1.1
            w[k] = 0.0
        elif 6.0 <= t[k] < 12.0:
            v[k] = 0.8
            w[k] = 0.35
        elif 12.0 <= t[k] < 16.0:
            v[k] = 0.0
            w[k] = -0.6
        elif 16.0 <= t[k] < 20.0:
            v[k] = 1.0
            w[k] = 0.15

    theta = np.zeros(N)
    for k in range(1, N):
        theta[k] = wrap_to_pi(theta[k - 1] + dt * w[k])
    return t, v, w, theta


def make_measurements(v_true: np.ndarray, w_true: np.ndarray, theta_true: np.ndarray, dt: float, seed: int = 2):
    rng = np.random.default_rng(seed)

    # Wheel measurements: noise + quantization + sporadic slip spikes
    v_w = v_true + 0.05 * rng.standard_normal(len(v_true))
    w_w = w_true + 0.08 * rng.standard_normal(len(w_true))

    # quantization effect (like finite encoder resolution, mapped to velocity)
    qv = 0.02
    qw = 0.02
    v_w = np.round(v_w / qv) * qv
    w_w = np.round(w_w / qw) * qw

    # Inject slip spikes
    spike_idx = rng.choice(len(v_true), size=12, replace=False)
    w_w[spike_idx] += rng.normal(loc=0.0, scale=2.0, size=len(spike_idx))
    v_w[spike_idx] += rng.normal(loc=0.0, scale=0.8, size=len(spike_idx))

    # Wheel heading from integrating wheel yaw rate (raw)
    theta_w = np.zeros_like(theta_true)
    for k in range(1, len(theta_w)):
        theta_w[k] = wrap_to_pi(theta_w[k - 1] + dt * w_w[k])

    # Gyro yaw rate: bias + noise + slow bias drift
    bias0 = 0.03
    bias_rw = 0.0005
    bias = np.zeros_like(w_true)
    bias[0] = bias0
    for k in range(1, len(bias)):
        bias[k] = bias[k - 1] + bias_rw * rng.standard_normal()
    w_g = w_true + bias + 0.05 * rng.standard_normal(len(w_true))

    return v_w, w_w, theta_w, w_g


def main():
    dt = 0.01
    T = 20.0

    t, v_true, w_true, theta_true = simulate_truth(T, dt)
    v_w, w_w, theta_w, w_g = make_measurements(v_true, w_true, theta_true, dt)

    # Robust pre-filter: Hampel suppresses spikes before low-pass
    hampel = HampelFilter(window=11, n_sigmas=3.0)
    v_w_h = hampel.apply(v_w)
    w_w_h = hampel.apply(w_w)

    cfg = OdometryFilterConfig(dt=dt)
    filt = OdometryFilter(cfg)

    x_f = np.zeros_like(t)
    y_f = np.zeros_like(t)
    th_f = np.zeros_like(t)

    for k in range(len(t)):
        x_f[k], y_f[k], th_f[k] = filt.step(v_w_h[k], w_w_h[k], w_g[k], theta_w[k])

    # Ground-truth position
    x_true = np.zeros_like(t)
    y_true = np.zeros_like(t)
    for k in range(1, len(t)):
        x_true[k] = x_true[k - 1] + dt * v_true[k] * math.cos(theta_true[k])
        y_true[k] = y_true[k - 1] + dt * v_true[k] * math.sin(theta_true[k])

    # Plots
    fig1 = plt.figure()
    plt.plot(t, v_true, label="v true")
    plt.plot(t, v_w, label="v wheel raw", alpha=0.5)
    plt.plot(t, v_w_h, label="v wheel Hampel", alpha=0.8)
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.legend()
    plt.title("Linear velocity filtering")

    fig2 = plt.figure()
    plt.plot(t, w_true, label="w true")
    plt.plot(t, w_w, label="w wheel raw", alpha=0.5)
    plt.plot(t, w_w_h, label="w wheel Hampel", alpha=0.8)
    plt.plot(t, w_g, label="w gyro", alpha=0.6)
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.legend()
    plt.title("Yaw-rate streams (wheel vs gyro)")

    fig3 = plt.figure()
    plt.plot(x_true, y_true, label="truth")
    plt.plot(x_f, y_f, label="filtered odometry")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.title("Trajectory comparison")

    fig4 = plt.figure()
    e_th = np.array([wrap_to_pi(th_f[k] - theta_true[k]) for k in range(len(t))])
    plt.plot(t, e_th)
    plt.xlabel("t [s]")
    plt.ylabel("heading error [rad]")
    plt.title("Heading error after complementary fusion")

    plt.show()


if __name__ == "__main__":
    main()
