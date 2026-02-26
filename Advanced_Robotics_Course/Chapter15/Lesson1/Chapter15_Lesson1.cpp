#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

double phi_prime(double r, double a, double b, double eps) {
    return 2.0 * a * r - b / (r + eps);
}

int main() {
    const int N = 20;
    const double a = 0.5;
    const double b = 1.0;
    const double eps = 1e-2;
    const double R = 1.0;
    const double h = 0.02;
    const int steps = 1000;

    std::vector<Vec2> x(N);
    std::vector<Vec2> x_new(N);

    // Simple initialization
    for (int i = 0; i < N; ++i) {
        x = -2.0 + 4.0 * (double)std::rand() / RAND_MAX;
        x = -2.0 + 4.0 * (double)std::rand() / RAND_MAX;
    }

    for (int k = 0; k < steps; ++k) {
        for (int i = 0; i < N; ++i) {
            Vec2 force(0.0, 0.0);
            for (int j = 0; j < N; ++j) {
                if (i == j) continue;
                Vec2 diff = x[i] - x[j];
                double dist = diff.norm();
                if (dist < 1e-6 || dist > R) continue;
                double fmag = phi_prime(dist, a, b, eps);
                force += (fmag / dist) * diff;
            }
            x_new[i] = x[i] - h * force;
        }
        x = x_new;
    }

    // Output final positions
    for (int i = 0; i < N; ++i) {
        std::cout << i << " : " << x[i].transpose() << std::endl;
    }
    return 0;
}
      
