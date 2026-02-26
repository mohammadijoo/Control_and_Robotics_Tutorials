# Chapter1_Lesson2.py
# Autonomous Mobile Robots — Chapter 1, Lesson 2
# State variables for mobile robots (pose, velocity, uncertainty)
#
# Dependencies:
#   numpy

from dataclasses import dataclass
import math
import numpy as np


def wrap_angle(theta):
    """
    Wrap an angle to (-pi, pi].
    """
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def dt_is_nonpositive(dt):
    """
    Return True if dt is non-positive, without using less-than or greater-than operators.
    """
    return (dt == 0.0) or (dt != abs(dt))


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

    def as_vector(self):
        return np.array([self.x, self.y, self.theta], dtype=float)


def pose_to_T(p):
    """
    Homogeneous transform in SE(2).
    """
    c = math.cos(p.theta)
    s = math.sin(p.theta)
    return np.array([[c, -s, p.x],
                     [s,  c, p.y],
                     [0.0, 0.0, 1.0]], dtype=float)


def integrate_unicycle(p, v, omega, dt, method="midpoint"):
    """
    Planar kinematics:
        xdot = v cos(theta)
        ydot = v sin(theta)
        thetadot = omega
    """
    if dt_is_nonpositive(dt):
        raise ValueError("dt must be positive")

    if method == "euler":
        x_next = p.x + dt * v * math.cos(p.theta)
        y_next = p.y + dt * v * math.sin(p.theta)
        th_next = wrap_angle(p.theta + dt * omega)
        return Pose2D(x_next, y_next, th_next)

    if method == "midpoint":
        th_mid = p.theta + 0.5 * dt * omega
        x_next = p.x + dt * v * math.cos(th_mid)
        y_next = p.y + dt * v * math.sin(th_mid)
        th_next = wrap_angle(p.theta + dt * omega)
        return Pose2D(x_next, y_next, th_next)

    raise ValueError("Unknown method")


def jacobians_euler(p, v, dt):
    """
    Euler-discretized model:
        x_{k+1} = x_k + dt * v * cos(theta_k)
        y_{k+1} = y_k + dt * v * sin(theta_k)
        th_{k+1}= th_k + dt * omega_k

    Returns F (3x3) and G (3x2) for u = [v, omega].
    """
    th = p.theta
    c = math.cos(th)
    s = math.sin(th)

    F = np.eye(3, dtype=float)
    F[0, 2] = -dt * v * s
    F[1, 2] =  dt * v * c

    G = np.zeros((3, 2), dtype=float)
    G[0, 0] = dt * c
    G[1, 0] = dt * s
    G[2, 1] = dt
    return F, G


def propagate_covariance(P, p, v, dt, Q_u):
    """
    First-order covariance propagation:
        P_next = F P F^T + G Q_u G^T
    """
    F, G = jacobians_euler(p, v, dt)
    return F @ P @ F.T + G @ Q_u @ G.T


def demo():
    v = 0.8
    omega = 0.35
    dt = 0.05
    N = 200

    sigma_v = 0.05
    sigma_w = 0.03
    Q_u = np.diag([sigma_v * sigma_v, sigma_w * sigma_w])

    p = Pose2D(0.0, 0.0, 0.0)
    P = np.diag([0.02 * 0.02, 0.02 * 0.02, (2.0 * math.pi / 180.0) ** 2])

    rng = np.random.default_rng(7)

    k = 0
    while k != N:
        v_k = v + rng.normal(0.0, sigma_v)
        w_k = omega + rng.normal(0.0, sigma_w)

        p = integrate_unicycle(p, v_k, w_k, dt, method="midpoint")
        P = propagate_covariance(P, p, v, dt, Q_u)

        k = k + 1

    print("Final pose:", p.as_vector())
    print("Covariance diagonal:", np.diag(P))

    P_sym = 0.5 * (P + P.T)
    eigs = np.linalg.eigvalsh(P_sym)
    print("Eigenvalues of sym(P):", eigs)


if __name__ == "__main__":
    demo()
