
#include <iostream>
#include <cmath>
#include <vector>

int main() {
    // Physical parameters
    const double m = 1.0;
    const double k_env = 5000.0;
    const double F_safe = 80.0;
    const double a_brake = 30.0;
    const double v_max = 1.0;

    // Derived
    const double v_force = F_safe / std::sqrt(m * k_env);

    // Simulation
    const double h = 0.0005;
    const double T = 1.0;
    const int N = static_cast<int>(T / h);

    // Controller parameters
    const double x_target = 0.0;
    const double k_p = 50.0;
    const double k_d = 5.0;
    const double k_v = 200.0;

    double x = 0.25;
    double v = 0.0;

    std::vector<double> xs(N), vs(N), Fs(N);

    for (int k = 0; k < N; ++k) {
        double d = x;
        double v_des = -k_p * (x - x_target) - k_d * v;

        // Distance-based bound
        double v_brake = std::sqrt(std::max(0.0, 2.0 * a_brake * d));
        double v_safe_bound = std::min(v_force, std::min(v_brake, v_max));

        double v_safe_cmd = v_des;
        if (v_des < -v_safe_bound) {
            v_safe_cmd = -v_safe_bound;
        }

        // Velocity-tracking torque
        double u = m * k_v * (v_safe_cmd - v);

        // Integrate
        v += (h / m) * u;
        x += h * v;

        // Contact force
        double F = 0.0;
        if (x < 0.0) {
            F = -k_env * x;
        }

        xs[k] = x;
        vs[k] = v;
        Fs[k] = F;
    }

    double max_penetration = xs[0];
    double max_force = std::fabs(Fs[0]);
    for (int k = 0; k < N; ++k) {
        if (xs[k] < max_penetration) max_penetration = xs[k];
        if (std::fabs(Fs[k]) > max_force) max_force = std::fabs(Fs[k]);
    }

    std::cout << "Max penetration (m): " << max_penetration << std::endl;
    std::cout << "Max contact force (N): " << max_force << std::endl;
    std::cout << "Theoretical bound F_safe (N): " << F_safe << std::endl;

    return 0;
}
