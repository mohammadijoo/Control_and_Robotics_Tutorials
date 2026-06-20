"""
Chapter18_Lesson1.py
Autonomous Mobile Robots (Control Engineering) — Chapter 18, Lesson 1
GPS/RTK Integration in Navigation (didactic EKF fusion example)

This script implements a lightweight EKF that fuses:
  - wheel speed (as an input / pseudo-measurement)
  - gyro yaw-rate (as an input with bias state)
  - GNSS/RTK position in a local ENU frame (as a measurement)

It is intentionally dependency-light (numpy only) and designed for clarity.
"""

from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    # map -pi to +pi for consistency
    return a if a > -math.pi else a + 2.0 * math.pi


@dataclass
class EkfConfig:
    # Process noise (continuous-time-ish, scaled by dt in discretization)
    q_xy: float = 0.05          # m^2/s (unmodeled planar drift)
    q_yaw: float = 0.01         # rad^2/s (yaw random walk beyond gyro)
    q_v: float = 0.5            # (m/s)^2/s (speed random walk)
    q_bg: float = 1e-4          # (rad/s)^2/s (gyro-bias random walk)

    # Wheel speed "measurement" noise (if you choose to update v)
    r_v: float = 0.2**2         # (m/s)^2

    # Default GNSS measurement covariance (overridden by RTK status)
    r_gps_fix: float = 1.5**2   # m^2 (single-point)
    r_gps_rtk: float = 0.02**2  # m^2 (RTK-fixed)

    # Innovation gate: chi-square threshold for 2D position (dof=2)
    # 0.99-quantile for chi2(2) ≈ 9.2103
    gate_chi2_dof2: float = 9.2103


class EkfGpsFusion:
    """
    State: x = [px, py, yaw, v, b_g]^T
      px, py  : position in ENU (meters)
      yaw     : heading in ENU frame (rad)
      v       : forward speed (m/s)
      b_g     : gyro z-bias (rad/s)

    Inputs:
      v_meas      : wheel speed estimate (m/s)
      omega_meas  : gyro yaw-rate (rad/s)

    Measurements:
      z_gnss = [px, py]^T
      (optionally) z_v = v_meas as pseudo-measurement for v
    """

    def __init__(self, cfg: EkfConfig = EkfConfig()):
        self.cfg = cfg
        self.x = np.zeros((5, 1), dtype=float)
        self.P = np.diag([10.0, 10.0, (20.0*math.pi/180.0)**2, 2.0**2, (5.0*math.pi/180.0)**2])

    def set_state(self, px: float, py: float, yaw: float, v: float, bg: float) -> None:
        self.x[:, 0] = [px, py, yaw, v, bg]

    def predict(self, v_meas: float, omega_meas: float, dt: float) -> None:
        if dt <= 0.0:
            return

        px, py, yaw, v, bg = self.x[:, 0]
        omega = omega_meas - bg

        # Simple speed blending: treat wheel speed as input that nudges v state
        # (You can instead do a separate measurement update for v.)
        v_pred = v  # random walk
        yaw_pred = wrap_to_pi(yaw + omega * dt)

        px_pred = px + v_pred * math.cos(yaw_pred) * dt
        py_pred = py + v_pred * math.sin(yaw_pred) * dt

        # State transition Jacobian F = df/dx
        F = np.eye(5)
        F[0, 2] = -v_pred * math.sin(yaw_pred) * dt
        F[0, 3] =  math.cos(yaw_pred) * dt
        F[1, 2] =  v_pred * math.cos(yaw_pred) * dt
        F[1, 3] =  math.sin(yaw_pred) * dt
        F[2, 4] = -dt  # yaw depends on -bg

        # Process noise covariance Q (discretized)
        q_xy = self.cfg.q_xy
        q_yaw = self.cfg.q_yaw
        q_v = self.cfg.q_v
        q_bg = self.cfg.q_bg
        Q = np.diag([q_xy*dt, q_xy*dt, q_yaw*dt, q_v*dt, q_bg*dt])

        # Commit prediction
        self.x[:, 0] = [px_pred, py_pred, yaw_pred, v_pred, bg]
        self.P = F @ self.P @ F.T + Q

        # Optional: incorporate wheel speed as a pseudo-measurement for v
        self.update_speed(v_meas)

    def update_speed(self, v_meas: float) -> None:
        # z = v + noise
        H = np.zeros((1, 5))
        H[0, 3] = 1.0
        z = np.array([[v_meas]], dtype=float)
        h = np.array([[self.x[3, 0]]], dtype=float)
        R = np.array([[self.cfg.r_v]], dtype=float)

        y = z - h
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_gnss(self, px_meas: float, py_meas: float, r_pos: float) -> bool:
        """
        GNSS/RTK position update with chi-square gating.
        Returns True if accepted, False if rejected by gate.
        """
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        z = np.array([[px_meas], [py_meas]], dtype=float)
        h = self.x[0:2, :]
        R = np.diag([r_pos, r_pos])

        y = z - h
        S = H @ self.P @ H.T + R
        d2 = float(y.T @ np.linalg.inv(S) @ y)  # Mahalanobis distance squared

        if d2 > self.cfg.gate_chi2_dof2:
            return False

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2, 0] = wrap_to_pi(float(self.x[2, 0]))
        self.P = (np.eye(5) - K @ H) @ self.P
        return True


# -----------------------------
# Minimal GNSS helpers (NMEA + local frame)
# -----------------------------
def nmea_degmin_to_deg(dm: float) -> float:
    """Convert ddmm.mmmm (or dddmm.mmmm) to decimal degrees."""
    deg = int(dm // 100)
    minutes = dm - 100 * deg
    return deg + minutes / 60.0


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """WGS84 LLA -> ECEF (meters)."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    N = a / math.sqrt(1.0 - e2 * s * s)

    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1.0 - e2) + alt_m) * s
    return np.array([x, y, z], dtype=float)


def ecef_to_enu(ecef: np.ndarray, ref_lla: tuple[float, float, float]) -> np.ndarray:
    """ECEF -> ENU (meters) w.r.t. reference LLA."""
    lat0, lon0, alt0 = ref_lla
    ref = lla_to_ecef(lat0, lon0, alt0)
    dx = ecef - ref

    lat = math.radians(lat0)
    lon = math.radians(lon0)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)

    R = np.array([
        [-slon,        clon,       0.0],
        [-slat*clon,  -slat*slon,   clat],
        [ clat*clon,   clat*slon,   slat],
    ], dtype=float)
    return R @ dx


def parse_nmea_gga(line: str):
    """
    Parse $GPGGA/$GNGGA.
    Returns (lat_deg, lon_deg, alt_m, fix_quality) or None if parse fails.

    fix_quality: 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, ...
    """
    if not line.startswith("$") or "GGA" not in line:
        return None
    parts = line.strip().split(",")
    if len(parts) < 10:
        return None

    try:
        lat_dm = float(parts[2]) if parts[2] else float("nan")
        lat_hemi = parts[3].strip()
        lon_dm = float(parts[4]) if parts[4] else float("nan")
        lon_hemi = parts[5].strip()
        fix_q = int(parts[6]) if parts[6] else 0
        alt = float(parts[9]) if parts[9] else 0.0
    except ValueError:
        return None

    lat = nmea_degmin_to_deg(lat_dm)
    lon = nmea_degmin_to_deg(lon_dm)
    if lat_hemi == "S":
        lat *= -1.0
    if lon_hemi == "W":
        lon *= -1.0
    return lat, lon, alt, fix_q


def rtk_quality_to_rpos(cfg: EkfConfig, fix_quality: int) -> float:
    """Map NMEA GGA fix quality to a position variance (m^2)."""
    # Common: 4=RTK fixed, 5=RTK float. Some receivers also report 2=DGPS.
    if fix_quality == 4:
        return cfg.r_gps_rtk
    if fix_quality == 5:
        return (0.2 ** 2)  # float: decimeter-ish
    if fix_quality == 2:
        return (0.8 ** 2)  # DGPS
    return cfg.r_gps_fix


# -----------------------------
# Demonstration (simulation)
# -----------------------------
def simulate():
    cfg = EkfConfig()
    ekf = EkfGpsFusion(cfg)
    ekf.set_state(px=0.0, py=0.0, yaw=0.0, v=1.0, bg=0.0)

    dt = 0.05
    T = 60.0
    n = int(T / dt)

    # True motion: mild turn
    v_true = 1.2
    omega_true = 0.07
    bg_true = 0.02  # rad/s gyro bias

    # Reference origin for ENU (choose any; here, fake small-area origin)
    ref_lla = (52.0, 4.0, 0.0)

    xs = []
    for k in range(n):
        t = k * dt

        # True state propagation
        if k == 0:
            px, py, yaw = 0.0, 0.0, 0.0
        else:
            px, py, yaw = xs[-1][1], xs[-1][2], xs[-1][3]
        yaw = wrap_to_pi(yaw + omega_true * dt)
        px = px + v_true * math.cos(yaw) * dt
        py = py + v_true * math.sin(yaw) * dt

        # Wheel and gyro measurements
        v_meas = v_true + np.random.normal(0.0, 0.05)
        omega_meas = omega_true + bg_true + np.random.normal(0.0, 0.01)

        ekf.predict(v_meas=v_meas, omega_meas=omega_meas, dt=dt)

        # GNSS update at 5 Hz
        accepted = None
        if k % int(0.2 / dt) == 0:
            # Simulate RTK "fixed" most of the time, occasionally degrade
            fix_q = 4 if (k % 200 != 0) else 5
            r_pos = rtk_quality_to_rpos(cfg, fix_q)

            # GNSS position measurement
            gnss_px = px + np.random.normal(0.0, math.sqrt(r_pos))
            gnss_py = py + np.random.normal(0.0, math.sqrt(r_pos))

            accepted = ekf.update_gnss(gnss_px, gnss_py, r_pos=r_pos)

        xs.append((t, px, py, yaw, ekf.x[0,0], ekf.x[1,0], ekf.x[2,0], accepted))

    # Print a small tail of results
    print("t, true_x, true_y, true_yaw, est_x, est_y, est_yaw, gnss_accepted")
    for row in xs[-10:]:
        print("{:6.2f} {:8.3f} {:8.3f} {:7.3f} {:8.3f} {:8.3f} {:7.3f} {}".format(*row))


if __name__ == "__main__":
    simulate()
