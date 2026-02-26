import numpy as np
from dataclasses import dataclass

@dataclass
class ValidationMetrics:
    rmse: float
    vaf: float
    fit_percent: float
    r2: float
    aic: float
    bic: float

def compute_residuals(tau_meas: np.ndarray,
                      tau_pred: np.ndarray) -> np.ndarray:
    """
    tau_meas: shape (N, m)
    tau_pred: shape (N, m)
    returns residuals e = tau_meas - tau_pred, shape (N, m)
    """
    return tau_meas - tau_pred

def compute_basic_metrics(tau_meas: np.ndarray,
                          tau_pred: np.ndarray) -> ValidationMetrics:
    tau_meas = np.asarray(tau_meas)
    tau_pred = np.asarray(tau_pred)
    assert tau_meas.shape == tau_pred.shape
    N, m = tau_meas.shape

    e = tau_meas - tau_pred

    # RMSE over all joints and samples
    rmse = np.sqrt(np.mean(np.sum(e**2, axis=1)))

    # Flatten for convenience
    y = tau_meas.reshape(-1)
    y_hat = tau_pred.reshape(-1)
    e_flat = e.reshape(-1)

    # Means and variance
    y_mean = np.mean(y)
    e_mean = np.mean(e_flat)
    var_y = np.var(y, ddof=1)
    var_e = np.var(e_flat, ddof=1)

    # VAF (in percent)
    vaf = 100.0 * (1.0 - var_e / var_y)

    # Fit percentage
    num = np.linalg.norm(y - y_hat)
    den = np.linalg.norm(y - y_mean)
    fit_percent = 100.0 * (1.0 - num / den)

    # R^2
    tss = np.sum((y - y_mean)**2)
    rss = np.sum((y - y_hat)**2)
    r2 = 1.0 - rss / tss

    # AIC and BIC
    # Assume effective number of parameters p is known:
    p = 30  # example; use actual number of estimated dynamic parameters
    sigma2_hat = np.mean(e_flat**2)
    Nm = N * m
    aic = 2.0 * p + Nm * np.log(sigma2_hat)
    bic = p * np.log(Nm) + Nm * np.log(sigma2_hat)

    return ValidationMetrics(rmse, vaf, fit_percent, r2, aic, bic)

def residual_autocorrelation(e: np.ndarray, L: int) -> np.ndarray:
    """
    e: residuals, shape (N,) for a single joint
    L: maximum lag
    returns normalized autocorrelation rho_e(l) for l = 0..L
    """
    e = np.asarray(e).reshape(-1)
    N = e.size
    e_mean = np.mean(e)
    e0 = e - e_mean
    R0 = np.dot(e0, e0) / N
    rho = np.zeros(L + 1)
    rho[0] = 1.0
    for ell in range(1, L + 1):
        num = np.dot(e0[ell:], e0[:-ell]) / (N - ell)
        rho[ell] = num / R0
    return rho

# Example usage:
# tau_meas_val, tau_pred_val: arrays of shape (N, m) on validation trajectories
# metrics = compute_basic_metrics(tau_meas_val, tau_pred_val)
# rho_joint1 = residual_autocorrelation((tau_meas_val - tau_pred_val)[:, 0], L=50)
      
