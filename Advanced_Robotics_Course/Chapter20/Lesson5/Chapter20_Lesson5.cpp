#include <iostream>
#include <vector>
#include <cmath>

struct EpisodeMetrics {
    int success;
    double time;
    double energy;
};

struct EvalStats {
    double p_hat;
    double p_ci_low;
    double p_ci_high;
    double time_mean;
    double time_std;
    double energy_mean;
    double energy_std;
};

EvalStats compute_stats(const std::vector<EpisodeMetrics>& data) {
    const std::size_t N = data.size();
    double sum_success = 0.0;
    double sum_time = 0.0, sum_time2 = 0.0;
    double sum_energy = 0.0, sum_energy2 = 0.0;

    for (const auto& e : data) {
        sum_success += static_cast<double>(e.success);
        sum_time += e.time;
        sum_time2 += e.time * e.time;
        sum_energy += e.energy;
        sum_energy2 += e.energy * e.energy;
    }

    EvalStats out{};
    out.p_hat = sum_success / static_cast<double>(N);

    const double z = 1.96; // 95% CI
    const double se_p = std::sqrt(out.p_hat * (1.0 - out.p_hat) / static_cast<double>(N));
    out.p_ci_low = std::max(0.0, out.p_hat - z * se_p);
    out.p_ci_high = std::min(1.0, out.p_hat + z * se_p);

    out.time_mean = sum_time / static_cast<double>(N);
    out.energy_mean = sum_energy / static_cast<double>(N);

    if (N > 1) {
        const double var_time =
            (sum_time2 - static_cast<double>(N) * out.time_mean * out.time_mean)
            / static_cast<double>(N - 1);
        const double var_energy =
            (sum_energy2 - static_cast<double>(N) * out.energy_mean * out.energy_mean)
            / static_cast<double>(N - 1);
        out.time_std = std::sqrt(std::max(0.0, var_time));
        out.energy_std = std::sqrt(std::max(0.0, var_energy));
    }

    return out;
}

int main() {
    std::vector<EpisodeMetrics> log;
    // TODO: fill log from your simulator or robot runs
    EvalStats stats = compute_stats(log);
    std::cout << "Success rate: " << stats.p_hat
              << " CI95: [" << stats.p_ci_low
              << ", " << stats.p_ci_high << "]\n";
    return 0;
}
      
