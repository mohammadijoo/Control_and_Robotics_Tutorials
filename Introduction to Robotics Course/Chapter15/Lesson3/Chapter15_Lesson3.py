import math
from typing import Iterable

def reliability_exponential(lambda_d: float, t: float) -> float:
    """
    Reliability R(t) = P(T > t) for an exponential time-to-dangerous-failure
    with rate lambda_d (per hour, for example).
    """
    if lambda_d < 0.0 or t < 0.0:
        raise ValueError("lambda_d and t must be non-negative")
    return math.exp(-lambda_d * t)

def k_out_of_n_reliability(Rc: float, n: int, k: int) -> float:
    """
    Reliability of a k-out-of-n architecture with identical,
    independent channels each with reliability Rc at time t.
    """
    if not (0.0 <= Rc <= 1.0):
        raise ValueError("Rc must be in [0, 1]")
    if k < 1 or k > n:
        raise ValueError("Need 1 <= k <= n")
    # Binomial tail sum
    R_sys = 0.0
    for i in range(k, n + 1):
        comb = math.comb(n, i)
        R_sys += comb * (Rc ** i) * ((1.0 - Rc) ** (n - i))
    return R_sys

def pfd_avg_1oo1(lambda_d: float, T_I: float) -> float:
    """
    Average probability of failure on demand for 1oo1 with exponential failures.
    Exact expression.
    """
    if lambda_d < 0.0 or T_I <= 0.0:
        raise ValueError("lambda_d must be non-negative and T_I > 0")
    if lambda_d == 0.0:
        return 0.0
    return 1.0 - (1.0 - math.exp(-lambda_d * T_I)) / (lambda_d * T_I)

def pfd_avg_1oo1_approx(lambda_d: float, T_I: float) -> float:
    """
    Low-demand approximation PFD_avg ~= lambda_d * T_I / 2.
    """
    return 0.5 * lambda_d * T_I

def pfd_avg_1oo2_approx(lambda_d: float, T_I: float) -> float:
    """
    Low-demand approximation for 1oo2: PFD_avg ~= lambda_d^2 * T_I^2 / 3.
    """
    return (lambda_d ** 2) * (T_I ** 2) / 3.0

if __name__ == "__main__":
    # Example: compare 1oo1 and 1oo2 for a safety channel
    lambda_d = 1e-6      # dangerous failures per hour
    T_I = 8760.0         # proof test interval: 1 year

    Rc = reliability_exponential(lambda_d, T_I)
    R_1oo1 = k_out_of_n_reliability(Rc, n=1, k=1)
    R_1oo2 = k_out_of_n_reliability(Rc, n=2, k=1)

    print("Component reliability Rc(T_I) =", Rc)
    print("R_1oo1(T_I) =", R_1oo1)
    print("R_1oo2(T_I) =", R_1oo2)

    pfd1_exact = pfd_avg_1oo1(lambda_d, T_I)
    pfd1_approx = pfd_avg_1oo1_approx(lambda_d, T_I)
    pfd2_approx = pfd_avg_1oo2_approx(lambda_d, T_I)

    print("PFD_avg 1oo1 (exact)  =", pfd1_exact)
    print("PFD_avg 1oo1 (approx) =", pfd1_approx)
    print("PFD_avg 1oo2 (approx) =", pfd2_approx)
      
