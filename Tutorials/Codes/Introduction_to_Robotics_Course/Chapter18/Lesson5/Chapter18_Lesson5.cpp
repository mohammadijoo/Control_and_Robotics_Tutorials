#include <iostream>
#include <random>
#include <Eigen/Dense>

int main() {
    using Eigen::Matrix2d;
    using Eigen::Vector2d;

    double dt = 0.05;
    Matrix2d A;
    A << 1.0, dt,
          0.0, 1.0;
    Eigen::Vector2d B(0.5 * dt * dt, dt);
    Eigen::RowVector2d K;
    K << -2.0, -1.0;

    double p_max = 5.0;
    double v_max = 3.0;
    int T = 200;
    int N_trials = 5000;
    double sigma_w = 0.05;

    std::mt19937 gen(42);
    std::normal_distribution<double> normal(0.0, 1.0);
    std::uniform_real_distribution<double> unif_p(-4.0, 4.0);
    std::uniform_real_distribution<double> unif_v(-2.0, 2.0);

    auto simulate_trial = [&]() {
        Vector2d x(unif_p(gen), unif_v(gen));
        for (int k = 0; k < T; ++k) {
            double u = K * x;
            Vector2d w(sigma_w * normal(gen), sigma_w * normal(gen));
            x = A * x + B * u + w;
            if (std::abs(x(0)) > p_max || std::abs(x(1)) > v_max) {
                return 1;
            }
        }
        return 0;
    };

    int failures = 0;
    for (int i = 0; i < N_trials; ++i) {
        failures += simulate_trial();
    }
    double p_hat = static_cast<double>(failures) / N_trials;
    std::cout << "Empirical failure rate: " << p_hat << std::endl;
    return 0;
}
      
