import random

def simulate_workcell(K=1000, lam=0.7):
    """
    Simple discrete-time queue for a single robot.
    At each time step, arrivals ~ Bernoulli(lam) for simplicity.
    Service capacity = 1 job per step.
    """
    q = 0               # current queue length
    total_departures = 0
    total_q = 0         # for average queue length

    for k in range(K):
        # arrivals: at most one with probability lam
        arrivals = 1 if random.random() < lam else 0
        q += arrivals

        # service: robot processes one job if available
        if q > 0:
            q -= 1
            total_departures += 1

        total_q += q

    avg_q = total_q / K
    throughput = total_departures / K
    utilization = throughput  # here capacity = 1 job/step

    return {
        "avg_queue_length": avg_q,
        "throughput": throughput,
        "utilization": utilization,
    }

if __name__ == "__main__":
    stats = simulate_workcell(K=10000, lam=0.7)
    print("Average queue length:", stats["avg_queue_length"])
    print("Throughput:", stats["throughput"])
    print("Utilization:", stats["utilization"])
      
