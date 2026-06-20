#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

int main() {
    const int N = 10;
    const double mass = 0.05;
    const double k_spring = 50.0;
    const double damping = 0.1;
    const double g = 9.81;
    const double dt = 5e-4;
    const int steps = 20000;

    std::vector<Eigen::Vector2d> x(N), v(N);
    for (int i = 0; i < N; ++i) {
        x[i] = Eigen::Vector2d(0.05 * i, 0.0);
        v[i].setZero();
    }
    std::vector<double> rest_lengths(N - 1, 0.05);

    auto compute_forces =
        [&](const std::vector<Eigen::Vector2d> &x,
              const std::vector<Eigen::Vector2d> &v) {
            std::vector<Eigen::Vector2d> f(N, Eigen::Vector2d::Zero());
            for (int i = 0; i < N - 1; ++i) {
                Eigen::Vector2d dx = x[i + 1] - x[i];
                double L = dx.norm();
                if (L > 1e-6) {
                    Eigen::Vector2d dir = dx / L;
                    Eigen::Vector2d fs = k_spring * (L - rest_lengths[i]) * dir;
                    f[i] += fs;
                    f[i + 1] -= fs;
                }
            }
            for (int i = 0; i < N; ++i) {
                f[i] -= damping * v[i];
                f[i].y() -= g * mass;
            }
            return f;
        };

    for (int step = 0; step < steps; ++step) {
        auto f = compute_forces(x, v);

        // Anchor first node
        f[0].setZero();
        v[0].setZero();
        x[0] = Eigen::Vector2d(0.0, 0.0);

        // Drive last node (gripper)
        double t = step * dt;
        Eigen::Vector2d target(0.05 * (N - 1),
                               0.1 * std::sin(2.0 * M_PI * t));
        x[N - 1] = target;
        v[N - 1].setZero();

        for (int i = 0; i < N; ++i) {
            Eigen::Vector2d a = f[i] / mass;
            v[i] += dt * a;
            x[i] += dt * v[i];
        }
    }

    std::cout << "Simulation finished." << std::endl;
    return 0;
}
      
