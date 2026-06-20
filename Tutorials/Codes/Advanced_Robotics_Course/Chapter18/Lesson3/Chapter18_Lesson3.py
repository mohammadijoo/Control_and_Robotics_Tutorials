import numpy as np

# Example: stress scenarios over friction, payload mass, and control latency.
# In practice, replace simulate_trial(...) with calls into PyBullet, MoveIt, etc.

rng = np.random.default_rng(seed=42)

def sample_stress_scenarios(N):
    """Draw N stress scenarios.
    friction: biased toward low values using a Beta distribution.
    mass: log-normal (kg).
    latency: exponential tail (s).
    """
    friction = rng.beta(a=0.5, b=2.0, size=N)        # more mass near 0
    mass = rng.lognormal(mean=0.0, sigma=0.25, size=N)
    latency = rng.exponential(scale=0.08, size=N)    # mean 80 ms
    return friction, mass, latency

def simulate_trial(friction, mass, latency):
    """Return (failed, taxonomy_label).

    failed: bool
    taxonomy_label: string representing failure class, e.g.
      "control:overshoot", "planning:infeasible", "sensing:occlusion", ...
    Here we implement a toy algebraic model instead of a real simulator.
    """
    # Constraint 1: stopping distance constraint violated if
    # effective friction * mass is too small.
    stopping_violation = (friction * mass) < 0.6

    # Constraint 2: closed-loop stability degrades if latency is too large.
    latency_violation = latency > 0.15  # seconds

    if stopping_violation and latency_violation:
        return True, "control+environment:multi-factor"
    if stopping_violation:
        return True, "control:insufficient_braking"
    if latency_violation:
        return True, "control:latency_instability"

    # Example of a non-safety failure: task time-out.
    timeout_violation = (mass > 2.5) and (latency > 0.10)
    if timeout_violation:
        return True, "performance:timeout"

    return False, "success"

def run_stress_test(N, delta=0.05):
    friction, mass, latency = sample_stress_scenarios(N)

    failures = 0
    labels = []
    for f, m, L in zip(friction, mass, latency):
        failed, label = simulate_trial(f, m, L)
        if failed:
            failures += 1
            labels.append(label)

    p_hat = failures / N
    # Hoeffding half-width for the two-sided confidence interval
    eps = np.sqrt(np.log(2.0 / delta) / (2.0 * N))
    ci_low = max(0.0, p_hat - eps)
    ci_high = min(1.0, p_hat + eps)

    # Empirical taxonomy distribution on failures
    from collections import Counter
    counts = Counter(labels)
    tax_probs = {k: v / failures for k, v in counts.items()} if failures > 0 else {}

    return {
        "N": N,
        "failures": failures,
        "p_hat": p_hat,
        "ci": (ci_low, ci_high),
        "taxonomy_distribution": tax_probs,
    }

if __name__ == "__main__":
    result = run_stress_test(N=5000, delta=0.05)
    print("N:", result["N"])
    print("Estimated failure probability:", result["p_hat"])
    print("95% Hoeffding CI:", result["ci"])
    print("Failure taxonomy distribution:")
    for k, v in result["taxonomy_distribution"].items():
        print(f"  {k}: {v:.3f}")
      
