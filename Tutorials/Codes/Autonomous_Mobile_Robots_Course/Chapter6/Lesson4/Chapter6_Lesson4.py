# Chapter6_Lesson4.py
# Independence Assumptions and Their Limits — correlated-sensor example for Bayesian fusion
#
# This script demonstrates how assuming conditional independence between measurements
# can lead to overconfident posteriors when sensor noises are correlated.
#
# Dependencies: numpy (standard). Optional: matplotlib for plotting.
#
# Usage:
#   python Chapter6_Lesson4.py

import numpy as np

def posterior_from_two_measurements_correlated(mu0, sigma0, y, sigma, rho):
    """
    Prior: x ~ N(mu0, sigma0^2)
    Measurements: y = [y1, y2]^T = [x, x]^T + v,  v ~ N(0, Sigma)
    Sigma = sigma^2 [[1, rho], [rho, 1]]
    Returns posterior mean and variance (scalar).
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    Sigma = (sigma**2) * np.array([[1.0, rho], [rho, 1.0]], dtype=float)
    # Information form: posterior precision = prior precision + H^T Sigma^{-1} H
    # Here H = [1, 1]^T (2x1), so H^T Sigma^{-1} H is scalar.
    Sinv = np.linalg.inv(Sigma)
    info_meas = np.array([1.0, 1.0]) @ Sinv @ np.array([1.0, 1.0])
    post_var = 1.0 / (1.0/(sigma0**2) + info_meas)
    # mean: mu = var*( mu0/sigma0^2 + H^T Sigma^{-1} y )
    info_vec = np.array([1.0, 1.0]) @ Sinv @ y
    post_mean = post_var * (mu0/(sigma0**2) + info_vec)
    return post_mean, post_var

def posterior_from_two_measurements_independent(mu0, sigma0, y, sigma):
    """
    Same model but with independence assumption rho=0, i.e., Sigma = sigma^2 I.
    Equivalent to multiplying two independent likelihoods N(y_i; x, sigma^2).
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    # Closed form: posterior precision = 1/sigma0^2 + 2/sigma^2
    post_var = 1.0 / (1.0/(sigma0**2) + 2.0/(sigma**2))
    post_mean = post_var * (mu0/(sigma0**2) + np.sum(y)/(sigma**2))
    return post_mean, post_var

def demo():
    mu0, sigma0 = 0.0, 2.0
    sigma = 1.0
    y1, y2 = 1.0, 1.2
    y = [y1, y2]

    print("Prior: x ~ N(mu0, sigma0^2) with mu0=%.3f, sigma0=%.3f" % (mu0, sigma0))
    print("Measurements: y1=%.3f, y2=%.3f, each with sigma=%.3f" % (y1, y2, sigma))
    print()

    for rho in [0.0, 0.3, 0.6, 0.9]:
        m_corr, v_corr = posterior_from_two_measurements_correlated(mu0, sigma0, y, sigma, rho)
        m_ind, v_ind = posterior_from_two_measurements_independent(mu0, sigma0, y, sigma)
        # Overconfidence factor: how much smaller the independent variance is vs correct
        overconf = v_ind / v_corr
        print("rho=%.1f:" % rho)
        print("  Correct correlated posterior: mean=%.6f, var=%.6f" % (m_corr, v_corr))
        print("  Indep. assumption posterior:  mean=%.6f, var=%.6f" % (m_ind, v_ind))
        print("  Variance ratio (indep/correct) = %.6f ( < 1 means overconfident )" % overconf)
        print()

    # Optional visualization (safe if matplotlib exists)
    try:
        import matplotlib.pyplot as plt
        rhos = np.linspace(0.0, 0.95, 20)
        vars_corr = []
        vars_ind = []
        for rho in rhos:
            _, v_corr = posterior_from_two_measurements_correlated(mu0, sigma0, y, sigma, rho)
            _, v_ind = posterior_from_two_measurements_independent(mu0, sigma0, y, sigma)
            vars_corr.append(v_corr)
            vars_ind.append(v_ind)
        plt.figure()
        plt.plot(rhos, vars_corr, label="Correct correlated")
        plt.plot(rhos, vars_ind, label="Assume independent (rho=0)")
        plt.xlabel("Correlation rho")
        plt.ylabel("Posterior variance")
        plt.title("Effect of ignoring correlation on posterior variance")
        plt.legend()
        plt.grid(True)
        plt.show()
    except Exception as e:
        print("Plot skipped (matplotlib not available or backend issue):", e)

if __name__ == "__main__":
    demo()
