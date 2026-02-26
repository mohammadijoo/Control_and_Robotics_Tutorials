#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

// Simple struct for configuration (could be loaded from JSON/YAML)
struct Config {
    double Kp;
    double Ki;
    double dt;
    double t_final;
    double settling_band;
};

// Closed-loop simulation for G(s) = 1 / (s + 1)^2 with PI controller
// State-space realization (minimal) of plant with input u and output y:
// x_dot = A x + B u, y = C x
// For G(s) = 1 / (s + 1)^2, one possible realization is:
// A = [ -1  1; 0  -1 ], B = [0; 1], C = [1  0]
struct PlantPI {
    Config cfg;
    double x1, x2;  // plant states
    double xI;      // integral state for PI

    explicit PlantPI(const Config& c)
        : cfg(c), x1(0.0), x2(0.0), xI(0.0) {}

    void reset() {
        x1 = 0.0; x2 = 0.0; xI = 0.0;
    }

    // One integration step using forward Euler for simplicity
    double step(double r) {
        double y = x1;
        double e = r - y;
        xI += e * cfg.dt;

        double u = cfg.Kp * e + cfg.Ki * xI;

        double x1_dot = -x1 + x2;
        double x2_dot = -x2 + u;

        x1 += x1_dot * cfg.dt;
        x2 += x2_dot * cfg.dt;
        return y;
    }
};

int main() {
    Config cfg;
    cfg.Kp = 4.0;
    cfg.Ki = 3.0;
    cfg.dt = 0.001;
    cfg.t_final = 10.0;
    cfg.settling_band = 0.02;

    PlantPI system(cfg);
    const int n_steps = static_cast<int>(cfg.t_final / cfg.dt);
    std::vector<double> t_vec;
    std::vector<double> y_vec;
    t_vec.reserve(n_steps + 1);
    y_vec.reserve(n_steps + 1);

    double t = 0.0;
    for (int k = 0; k <= n_steps; ++k) {
        double y = system.step(1.0);  // unit step
        t_vec.push_back(t);
        y_vec.push_back(y);
        t += cfg.dt;
    }

    // Compute simple metrics
    double y_final = y_vec.back();
    double Mp = 0.0;
    for (double y : y_vec) {
        if (y - 1.0 > Mp) Mp = y - 1.0;
    }
    double ess = 1.0 - y_final;

    double t_s = 0.0;
    for (int k = n_steps; k >= 0; --k) {
        if (std::fabs(y_vec[static_cast<std::size_t>(k)] - 1.0) > cfg.settling_band) {
            t_s = t_vec[static_cast<std::size_t>(k)];
            break;
        }
    }

    // Persist results (CSV for plots, text for metrics)
    std::ofstream data_file("cpp_pi_closed_loop.csv");
    data_file << "t,y\n";
    for (std::size_t k = 0; k < t_vec.size(); ++k) {
        data_file << t_vec[k] << "," << y_vec[k] << "\n";
    }

    std::ofstream metrics_file("cpp_pi_metrics.txt");
    metrics_file << "Kp=" << cfg.Kp << "\n";
    metrics_file << "Ki=" << cfg.Ki << "\n";
    metrics_file << "Mp=" << Mp << "\n";
    metrics_file << "ess=" << ess << "\n";
    metrics_file << "ts=" << t_s << "\n";

    std::cout << "Results written to cpp_pi_closed_loop.csv and cpp_pi_metrics.txt\n";

    // In a robotics stack, the same simulation could be wrapped in a ROS node,
    // with cfg loaded from and written back to the ROS parameter server to
    // guarantee that all experiments are driven by explicit parameters.
    return 0;
}
