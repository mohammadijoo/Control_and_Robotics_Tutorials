# Chapter7_Lesson5_Ex1.py
# Exercise: NIS gating for wheel/GPS updates + basic consistency checks
# Dependencies: numpy

import numpy as np
import math

def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def nis(y: np.ndarray, S: np.ndarray) -> float:
    return float(y.T @ np.linalg.inv(S) @ y)

def chi2_threshold(dof: int, alpha: float) -> float:
    """
    Returns a rough Chi-square threshold using Wilson-Hilferty approximation.
    Avoids requiring SciPy; adequate for teaching demos.
    """
    # X ~ Chi2(k). Approx quantile via normal quantile and transform.
    # q ≈ k*(1 - 2/(9k) + z*sqrt(2/(9k)))^3
    # z is normal quantile; approximate using inverse-erf.
    def inv_norm_cdf(p):
        # Acklam-like approximation (coarse); for p in (0,1)
        # For lab use only.
        a1=-3.969683028665376e+01; a2=2.209460984245205e+02; a3=-2.759285104469687e+02
        a4=1.383577518672690e+02; a5=-3.066479806614716e+01; a6=2.506628277459239e+00
        b1=-5.447609879822406e+01; b2=1.615858368580409e+02; b3=-1.556989798598866e+02
        b4=6.680131188771972e+01; b5=-1.328068155288572e+01
        c1=-7.784894002430293e-03; c2=-3.223964580411365e-01; c3=-2.400758277161838e+00
        c4=-2.549732539343734e+00; c5=4.374664141464968e+00; c6=2.938163982698783e+00
        d1=7.784695709041462e-03; d2=3.224671290700398e-01; d3=2.445134137142996e+00
        d4=3.754408661907416e+00
        plow=0.02425; phigh=1-plow
        if p < plow:
            q=math.sqrt(-2*math.log(p))
            return (((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6)/((((d1*q+d2)*q+d3)*q+d4)*q+1)
        if p > phigh:
            q=math.sqrt(-2*math.log(1-p))
            return -(((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6)/((((d1*q+d2)*q+d3)*q+d4)*q+1)
        q=p-0.5; r=q*q
        return (((((a1*r+a2)*r+a3)*r+a4)*r+a5)*r+a6)*q/(((((b1*r+b2)*r+b3)*r+b4)*r+b5)*r+1)

    z = inv_norm_cdf(1.0 - alpha)
    k = float(dof)
    return k * (1.0 - 2.0/(9.0*k) + z * math.sqrt(2.0/(9.0*k)))**3

# Example usage (plug into your EKF/UKF update):
if __name__ == "__main__":
    dof_wheel = 2
    alpha = 0.01
    gamma = chi2_threshold(dof_wheel, alpha)
    print("Gate threshold (Chi2 approx), dof=2, alpha=0.01:", gamma)

    # Suppose an innovation y and S
    y = np.array([0.5, -0.1])
    S = np.diag([0.2**2, 0.1**2])
    val = nis(y, S)
    print("NIS:", val, " -> accept?" , (val <= gamma))
