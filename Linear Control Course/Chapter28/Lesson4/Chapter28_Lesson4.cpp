#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

// Example: analytic metrics for second-order PD-controlled joint
struct Metrics {
    double zeta;
    double wn;
    bool stable;
};

Metrics metrics_from_params(double J, double B, double Kp, double Kd) {
    // Stability conditions for J s^2 + (B + Kd) s + Kp
    bool stable = (J > 0.0) && (B + Kd > 0.0) && (Kp > 0.0);

    double wn = std::sqrt(Kp / J);
    double zeta = (B + Kd) / (2.0 * std::sqrt(J * Kp));

    return {zeta, wn, stable};
}

int main() {
    std::vector<double> J_vals  = {0.8, 1.0, 1.2};
    std::vector<double> B_vals  = {0.3, 0.5, 0.7};
    std::vector<double> Kp_vals = {10.0, 20.0, 30.0};
    std::vector<double> Kd_vals = {0.5, 1.0, 2.0};

    double zeta_min = 0.5;
    double Mp_max   = 0.15;  // overshoot limit (fraction)
    double ts_max   = 2.0;   // approximate 2% settling time

    int total_count  = 0;
    int stable_count = 0;
    int robust_count = 0;

    for (double J : J_vals) {
        for (double B : B_vals) {
            for (double Kp : Kp_vals) {
                for (double Kd : Kd_vals) {
                    ++total_count;
                    Metrics m = metrics_from_params(J, B, Kp, Kd);

                    if (!m.stable) continue;
                    ++stable_count;

                    // Time-domain specs using standard second-order formulas
                    if (m.zeta <= 0.0 || m.zeta >= 1.0) continue;
                    double Mp = std::exp(-M_PI * m.zeta / std::sqrt(1.0 - m.zeta * m.zeta));
                    double ts = 4.0 / (m.zeta * m.wn);

                    if (m.zeta >= zeta_min && Mp <= Mp_max && ts <= ts_max) {
                        ++robust_count;
                    }
                }
            }
        }
    }

    std::cout << "Stable fraction: "
              << static_cast<double>(stable_count) / total_count << std::endl;
    std::cout << "Robust fraction: "
              << static_cast<double>(robust_count) / total_count << std::endl;

    return 0;
}
