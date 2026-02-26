#include <iostream>
#include <random>

struct Stats {
    double avg_queue_length;
    double throughput;
    double utilization;
};

Stats simulate_workcell(int K = 1000, double lam = 0.7) {
    std::mt19937 gen(42);
    std::bernoulli_distribution bern(lam);

    int q = 0;
    int total_departures = 0;
    long long total_q = 0;

    for (int k = 0; k < K; ++k) {
        int arrivals = bern(gen) ? 1 : 0;
        q += arrivals;

        if (q > 0) {
            q -= 1;
            total_departures += 1;
        }

        total_q += q;
    }

    Stats s;
    s.avg_queue_length = static_cast<double>(total_q) / K;
    s.throughput = static_cast<double>(total_departures) / K;
    s.utilization = s.throughput; // capacity = 1 job/step
    return s;
}

int main() {
    Stats s = simulate_workcell(10000, 0.7);
    std::cout << "Average queue length: " << s.avg_queue_length << "\n";
    std::cout << "Throughput: " << s.throughput << "\n";
    std::cout << "Utilization: " << s.utilization << "\n";
    return 0;
}
      
