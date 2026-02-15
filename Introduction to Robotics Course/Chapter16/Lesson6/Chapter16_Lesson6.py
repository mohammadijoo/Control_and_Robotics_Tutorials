import math

def series_reliability(lambdas, t):
    """
    lambdas: iterable of constant failure rates [1/h]
    t: mission time [h]
    """
    lam_eq = sum(lambdas)
    return math.exp(-lam_eq * t)

def parallel_reliability(lambdas, t):
    """
    Parallel system of independent exponential components.
    """
    # Probability each component fails by time t
    fail_probs = [1.0 - math.exp(-lam * t) for lam in lambdas]
    system_fail = 1.0
    for q in fail_probs:
        system_fail *= q
    return 1.0 - system_fail

def max_component_lambda(R_target, t, n):
    """
    Maximum allowable constant failure rate for n identical
    series components achieving R_target at time t.
    """
    if R_target <= 0.0 or R_target >= 1.0:
        raise ValueError("R_target must be in (0, 1)")
    lam_eq = -math.log(R_target) / t
    return lam_eq / n

def unit_cost(F, c_v, c_o, Y, N):
    """
    Unit cost model c_unit(N) = F/N + c_v/Y + c_o.
    """
    return F / N + c_v / Y + c_o

if __name__ == "__main__":
    # Example: 3 components in series, 1000 h mission
    lam = [1e-4, 2e-4, 1.5e-4]
    t_mission = 1000.0
    R_ser = series_reliability(lam, t_mission)
    R_par = parallel_reliability([1e-4, 1e-4], t_mission)
    print("Series reliability at 1000 h:", R_ser)
    print("Parallel redundancy (2 identical) at 1000 h:", R_par)

    # Cost comparison of designs A and B
    F_A, cA, Y_A, c_oA = 5e4, 250.0, 0.95, 40.0
    F_B, cB, Y_B, c_oB = 1.5e4, 280.0, 0.98, 35.0
    N = 2000
    print("c_A(N):", unit_cost(F_A, cA, c_oA, Y_A, N))
    print("c_B(N):", unit_cost(F_B, cB, c_oB, Y_B, N))
      
