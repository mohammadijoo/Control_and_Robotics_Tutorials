#include <random>
#include <iostream>
#include <string>

struct TrialResult {
    bool failed;
    std::string label;
};

// Toy stress model as in the Python example.
TrialResult simulate_trial(double friction, double mass, double latency) {
    bool stopping_violation = (friction * mass) < 0.6;
    bool latency_violation = latency > 0.15;
    bool timeout_violation = (mass > 2.5) && (latency > 0.10);

    if (stopping_violation && latency_violation) {
        return {true, "control+environment:multi-factor"};
    }
    if (stopping_violation) {
        return {true, "control:insufficient_braking"};
    }
    if (latency_violation) {
        return {true, "control:latency_instability"};
    }
    if (timeout_violation) {
        return {true, "performance:timeout"};
    }
    return {false, "success"};
}

int main() {
    const int N = 5000;
    const double delta = 0.05;

    std::mt19937 rng(42);
    std::gamma_distribution<double> gamma_f(0.5, 1.0);   // for friction
    std::lognormal_distribution<double> logn_m(0.0, 0.25);
    std::exponential_distribution<double> exp_lat(1.0 / 0.08);

    int failures = 0;

    for (int i = 0; i < N; ++i) {
        double friction = std::min(1.0, gamma_f(rng));  // truncated
        double mass = logn_m(rng);
        double latency = exp_lat(rng);

        TrialResult tr = simulate_trial(friction, mass, latency);
        if (tr.failed) {
            ++failures;
            // Here we could increment a map<std::string,int> to build taxonomy stats.
        }
    }

    double p_hat = static_cast<double>(failures) / N;
    double eps = std::sqrt(std::log(2.0 / delta) / (2.0 * N));
    double ci_low = std::max(0.0, p_hat - eps);
    double ci_high = std::min(1.0, p_hat + eps);

    std::cout << "N: " << N << "\n";
    std::cout << "Estimated failure probability: " << p_hat << "\n";
    std::cout << "Hoeffding 95% CI: [" << ci_low << ", " << ci_high << "]\n";

    return 0;
}
      
