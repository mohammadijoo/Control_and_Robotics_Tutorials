import math

def deployment_metrics(total_uptime_hours, n_failures, mttr_hours, mission_hours):
    if total_uptime_hours <= 0:
        raise ValueError("total_uptime_hours must be positive")
    if n_failures <= 0:
        # assume at least one virtual failure to get a conservative estimate
        n_failures = 1

    lam_hat = n_failures / total_uptime_hours  # failure rate lambda
    mtbf_hat = 1.0 / lam_hat
    availability_hat = mtbf_hat / (mtbf_hat + mttr_hours)
    mission_success = math.exp(-lam_hat * mission_hours)

    return {
        "lambda_hat": lam_hat,
        "MTBF_hat": mtbf_hat,
        "availability_hat": availability_hat,
        "mission_success_prob": mission_success,
    }

if __name__ == "__main__":
    metrics = deployment_metrics(
        total_uptime_hours=10000.0,
        n_failures=5,
        mttr_hours=2.0,
        mission_hours=8.0
    )
    for k, v in metrics.items():
        print(f"{k}: {v:.6f}")
      
