"""
Chapter7_Lesson2_Ex1.py
Exercise: Investigate how UKF scaling parameters (alpha, kappa, beta) affect localization RMSE.

This script reuses the UKF implementation from Chapter7_Lesson2.py by importing it
(if both files are in the same folder). It then runs multiple simulations and prints
RMSE statistics.

Run:
  python Chapter7_Lesson2_Ex1.py
"""

import math
import numpy as np

from Chapter7_Lesson2 import UKF, UKFParams, EKF, motion_model, measurement_model_factory, wrap_angle


def run_once(alpha: float, beta: float = 2.0, kappa: float = 0.0, seed: int = 0):
    np.random.seed(seed)
    dt = 0.1
    T = 200

    landmarks = np.array([
        [5.0,  0.0],
        [0.0,  6.0],
        [6.0,  6.0],
        [8.0, -2.0],
    ])

    x_true = np.array([0.0, 0.0, 0.2])

    ukf = UKF(dim_x=3, params=UKFParams(alpha=alpha, beta=beta, kappa=kappa))
    ukf.x = np.array([0.5, -0.5, -0.3])
    ukf.P = np.diag([0.8**2, 0.8**2, (20.0 * math.pi/180.0)**2])

    sigma_xy = 0.02
    sigma_th = math.radians(1.0)
    ukf.Q = np.diag([sigma_xy**2, sigma_xy**2, sigma_th**2])

    sigma_r = 0.15
    sigma_b = math.radians(2.0)
    R = np.diag([sigma_r**2, sigma_b**2])

    pos_errs = []

    for k in range(T):
        v = 1.0 + 0.2 * math.sin(0.07 * k)
        w = 0.35 * math.sin(0.03 * k)
        u = np.array([v, w])

        x_true = motion_model(x_true, u, dt)
        x_true += np.array([
            np.random.normal(0, sigma_xy),
            np.random.normal(0, sigma_xy),
            np.random.normal(0, sigma_th),
        ])
        x_true[2] = wrap_angle(float(x_true[2]))

        ukf.predict(lambda x, uu: motion_model(x, uu, dt), u)

        for lm in landmarks:
            h = measurement_model_factory(lm)
            z = h(x_true) + np.array([
                np.random.normal(0, sigma_r),
                np.random.normal(0, sigma_b),
            ])
            z[1] = wrap_angle(float(z[1]))
            ukf.update_sequential(z, h, R)

        pos_errs.append(float(np.linalg.norm(ukf.x[:2] - x_true[:2])))

    rmse = float(np.sqrt(np.mean(np.square(pos_errs))))
    return rmse


def main():
    alphas = [0.1, 0.2, 0.35, 0.5, 0.8, 1.0]
    for a in alphas:
        rmses = [run_once(alpha=a, seed=s) for s in range(5)]
        print(f"alpha={a:.2f}  RMSE mean={np.mean(rmses):.4f}  std={np.std(rmses):.4f}  per-run={rmses}")


if __name__ == "__main__":
    main()
