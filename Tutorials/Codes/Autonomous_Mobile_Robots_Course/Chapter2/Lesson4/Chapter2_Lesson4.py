# Chapter2_Lesson4.py
"""Autonomous Mobile Robots — Chapter 2 Lesson 4
Omnidirectional Bases (mecanum, Swedish wheels)

This script implements:
  1) 4-wheel mecanum inverse/forward kinematics (redundant; least-squares forward map)
  2) 3-wheel Swedish/omni base inverse/forward kinematics (square; exact inverse)
  3) A simple pose integrator for planar motion

Dependencies:
  - numpy (required)
  - matplotlib (optional, for plotting the trajectory)

Notes:
  - Kinematic model assumes ideal rolling in the constrained traction direction and free motion along rollers.
  - For mecanum wheels, the roller skew reduces the effective drive component; modeled via kappa = r*cos(alpha).
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


def pinv_svd(A: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """Moore–Penrose pseudoinverse via SVD.

    Returns A^+ such that:
      - A A^+ A = A
      - A^+ A A^+ = A^+
      - (A A^+)^T = A A^+
      - (A^+ A)^T = A^+ A
    """
    U, s, Vt = np.linalg.svd(A, full_matrices=False)
    s_inv = np.zeros_like(s)
    for i, si in enumerate(s):
        if si > tol:
            s_inv[i] = 1.0 / si
    return Vt.T @ np.diag(s_inv) @ U.T


def body_to_world(vx: float, vy: float, theta: float) -> tuple[float, float]:
    """Rotate body-frame planar velocity (vx,vy) into world frame."""
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    xdot = c * vx - s * vy
    ydot = s * vx + c * vy
    return xdot, ydot


@dataclass(frozen=True)
class Mecanum4:
    """4-wheel mecanum kinematics.

    Wheel order:
      1: Front-Left  (FL)
      2: Front-Right (FR)
      3: Rear-Left   (RL)
      4: Rear-Right  (RR)

    Geometry:
      - wheel radius r
      - wheel centers at (±lx, ±ly) in body frame
      - alpha: skew angle between traction direction and wheel tangential drive direction
              (alpha=45deg for classic mecanum)
    """
    r: float
    lx: float
    ly: float
    alpha: float = float(np.deg2rad(45.0))

    def jacobian(self) -> np.ndarray:
        a = self.lx + self.ly
        kappa = self.r * float(np.cos(self.alpha))  # effective drive coefficient
        if abs(kappa) < 1e-12:
            raise ValueError("kappa is ~0; check alpha and r.")
        # Mapping: w = (1/kappa) * A * [vx, vy, omega]^T
        A = np.array(
            [
                [1.0, -1.0, -a],  # FL
                [1.0,  1.0,  a],  # FR
                [1.0,  1.0, -a],  # RL
                [1.0, -1.0,  a],  # RR
            ],
            dtype=float,
        )
        return (1.0 / kappa) * A

    def inverse(self, vx: float, vy: float, omega: float) -> np.ndarray:
        v = np.array([vx, vy, omega], dtype=float)
        return self.jacobian() @ v

    def forward_ls(self, w: np.ndarray) -> np.ndarray:
        """Least-squares forward kinematics: v = argmin ||J v - w||_2."""
        J = self.jacobian()
        w = np.asarray(w, dtype=float).reshape(-1)
        if w.size != 4:
            raise ValueError("Expected 4 wheel speeds.")
        return pinv_svd(J) @ w

    def nullspace_vector(self) -> np.ndarray:
        """One convenient nullspace vector for classic mecanum configuration.

        If v = 0, wheel speeds proportional to this vector ideally yield zero chassis twist.
        """
        return np.array([1.0, -1.0, 1.0, -1.0], dtype=float)


@dataclass(frozen=True)
class Omni3:
    """3-wheel Swedish/omni base kinematics (non-redundant).

    We place 3 omni wheels on a circle of radius L at angles psi_i:
      psi = [0, 2pi/3, 4pi/3]
    Wheel traction direction is tangent to the circle:
      beta_i = psi_i + pi/2

    With ideal omni wheels, kappa = r (no skew reduction).
    """
    r: float
    L: float

    def jacobian(self) -> np.ndarray:
        psi = np.array([0.0, 2.0*np.pi/3.0, 4.0*np.pi/3.0], dtype=float)
        beta = psi + np.pi/2.0  # tangential traction direction
        x = self.L * np.cos(psi)
        y = self.L * np.sin(psi)

        kappa = self.r
        J = np.zeros((3, 3), dtype=float)
        for i in range(3):
            cb = float(np.cos(beta[i]))
            sb = float(np.sin(beta[i]))
            J[i, 0] = cb / kappa
            J[i, 1] = sb / kappa
            J[i, 2] = (-y[i]*cb + x[i]*sb) / kappa
        return J

    def inverse(self, vx: float, vy: float, omega: float) -> np.ndarray:
        v = np.array([vx, vy, omega], dtype=float)
        return self.jacobian() @ v

    def forward(self, w: np.ndarray) -> np.ndarray:
        J = self.jacobian()
        w = np.asarray(w, dtype=float).reshape(-1)
        if w.size != 3:
            raise ValueError("Expected 3 wheel speeds.")
        return np.linalg.solve(J, w)


def simulate_open_loop(base: Mecanum4 | Omni3, T: float = 8.0, dt: float = 0.01) -> np.ndarray:
    """Simple open-loop simulation: command a constant body twist and integrate pose.

    Pose state: [x, y, theta] in world frame.
    """
    # Example command (body frame): translate and rotate
    vx_cmd, vy_cmd, omega_cmd = 0.35, 0.20, 0.45

    n = int(np.ceil(T / dt))
    pose = np.zeros((n + 1, 3), dtype=float)

    # Compute wheel speeds, then reconstruct twist using forward map (mimics encoder-based odometry)
    w = base.inverse(vx_cmd, vy_cmd, omega_cmd)

    for k in range(n):
        if isinstance(base, Mecanum4):
            v_hat = base.forward_ls(w)
        else:
            v_hat = base.forward(w)

        vx, vy, omega = float(v_hat[0]), float(v_hat[1]), float(v_hat[2])
        x, y, th = pose[k, :]
        xdot, ydot = body_to_world(vx, vy, th)
        pose[k + 1, 0] = x + dt * xdot
        pose[k + 1, 1] = y + dt * ydot
        pose[k + 1, 2] = th + dt * omega

    return pose


def main() -> None:
    np.set_printoptions(precision=6, suppress=True)

    print("=== 4-wheel mecanum demo ===")
    mec = Mecanum4(r=0.05, lx=0.20, ly=0.15, alpha=float(np.deg2rad(45.0)))
    v_cmd = np.array([0.40, -0.10, 0.60], dtype=float)
    w = mec.inverse(*v_cmd)
    v_hat = mec.forward_ls(w)

    print("Commanded twist [vx, vy, omega] =", v_cmd)
    print("Wheel speeds [w1..w4] rad/s      =", w)
    print("Reconstructed twist (LS)        =", v_hat)
    print("Nullspace example vector        =", mec.nullspace_vector())

    pose = simulate_open_loop(mec, T=6.0, dt=0.01)
    print("Final pose [x, y, theta] =", pose[-1, :])

    # Optional plotting
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(pose[:, 0], pose[:, 1])
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Open-loop trajectory (mecanum)")
        plt.axis("equal")
        plt.show()
    except Exception as e:
        print("Plot skipped:", e)

    print("\n=== 3-wheel omni demo ===")
    omni = Omni3(r=0.05, L=0.18)
    v_cmd2 = np.array([0.20, 0.25, -0.40], dtype=float)
    w2 = omni.inverse(*v_cmd2)
    v_hat2 = omni.forward(w2)
    print("Commanded twist [vx, vy, omega] =", v_cmd2)
    print("Wheel speeds [w1..w3] rad/s      =", w2)
    print("Reconstructed twist (solve)      =", v_hat2)


if __name__ == "__main__":
    main()
