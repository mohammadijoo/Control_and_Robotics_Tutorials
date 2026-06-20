// Chapter 6 - Lesson 1: Belief as a Probability Distribution Over Pose
// Autonomous Mobile Robots (Control Engineering)
//
// This C++ example uses Eigen to represent a Gaussian belief over pose x = (x, y, theta),
// discretizes it onto a grid, normalizes by cell volume, and computes the expected pose.
// Theta is treated with wrap-to-pi for differences (Gaussian-on-R approximation).
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter6_Lesson1.cpp -I /usr/include/eigen3 -o Chapter6_Lesson1
//
// Note: install Eigen (header-only).

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

static double wrapToPi(double a) {
    // Wrap to (-pi, pi]
    const double TWO_PI = 2.0 * M_PI;
    a = std::fmod(a + M_PI, TWO_PI);
    if (a < 0.0) a += TWO_PI;
    return a - M_PI;
}

struct GaussianBeliefSE2 {
    Eigen::Vector3d mu;
    Eigen::Matrix3d Sigma;

    double pdf(const Eigen::Vector3d& x) const {
        Eigen::Vector3d dx = x - mu;
        dx(2) = wrapToPi(dx(2));

        const double detS = Sigma.determinant();
        const Eigen::Matrix3d invS = Sigma.inverse();
        const double norm = 1.0 / std::sqrt(std::pow(2.0 * M_PI, 3) * detS);
        const double expo = -0.5 * (dx.transpose() * invS * dx)(0, 0);
        return norm * std::exp(expo);
    }
};

int main() {
    // Gaussian belief parameters
    GaussianBeliefSE2 g;
    g.mu << 2.0, -1.0, 0.7;
    g.Sigma.setZero();
    g.Sigma(0, 0) = 0.2 * 0.2;
    g.Sigma(1, 1) = 0.3 * 0.3;
    g.Sigma(2, 2) = std::pow(10.0 * M_PI / 180.0, 2);

    Eigen::Vector3d x_test(2.1, -1.2, 0.75);
    std::cout << "pdf(x_test) = " << g.pdf(x_test) << "\n";

    // Grid definition
    const int Nx = 101, Ny = 101, Nt = 121;
    const double x0 = 1.0, x1 = 3.0;
    const double y0 = -2.0, y1 = 0.0;
    const double th0 = -M_PI, th1 = M_PI; // endpoint excluded for uniform bins

    std::vector<double> xs(Nx), ys(Ny), ths(Nt);
    for (int i = 0; i < Nx; ++i) xs[i] = x0 + (x1 - x0) * i / (Nx - 1);
    for (int j = 0; j < Ny; ++j) ys[j] = y0 + (y1 - y0) * j / (Ny - 1);
    for (int k = 0; k < Nt; ++k) ths[k] = th0 + (th1 - th0) * k / Nt;

    const double dx = xs[1] - xs[0];
    const double dy = ys[1] - ys[0];
    const double dth = ths[1] - ths[0];
    const double cellVol = dx * dy * dth;

    // Evaluate and normalize
    std::vector<double> b(Nx * Ny * Nt, 0.0);
    auto idx = [Ny, Nt](int i, int j, int k) { return (i * Ny + j) * Nt + k; };

    double Z = 0.0;
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nt; ++k) {
                Eigen::Vector3d x(xs[i], ys[j], ths[k]);
                const double val = g.pdf(x);
                b[idx(i, j, k)] = val;
                Z += val * cellVol;
            }
        }
    }
    for (double& v : b) v /= Z;

    double check = 0.0;
    for (double v : b) check += v * cellVol;
    std::cout << "grid normalization check: " << check << "\n";

    // Expectation: E[x], E[y], circular mean of theta
    double ex = 0.0, ey = 0.0, esin = 0.0, ecos = 0.0;
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nt; ++k) {
                const double w = b[idx(i, j, k)] * cellVol;
                ex += xs[i] * w;
                ey += ys[j] * w;
                esin += std::sin(ths[k]) * w;
                ecos += std::cos(ths[k]) * w;
            }
        }
    }
    const double eth = std::atan2(esin, ecos);
    std::cout << "grid E[pose] = [" << ex << ", " << ey << ", " << eth << "]\n";

    return 0;
}
