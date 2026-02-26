// Chapter6_Lesson5.cpp
// Lab: Build a Basic Bayes Filter Loop (discrete 1D cyclic state)
//
// Dependencies: Eigen (header-only). Compile example (Linux/macOS):
//   g++ -O2 -std=c++17 Chapter6_Lesson5.cpp -I /usr/include/eigen3 -o bayes_loop
//
// This file implements:
//   bel_bar = T(u) * bel
//   bel     = normalize( bel_bar .* l(z) )
//
// where T(u) is a cyclic convolution operator parameterized by a Gaussian kernel.

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>

static double wrap_interval(double x, double half_period) {
    double period = 2.0 * half_period;
    double y = std::fmod(x + half_period, period);
    if (y < 0.0) y += period;
    return y - half_period;
}

static double gaussian_pdf(double x, double mu, double sigma) {
    const double c = 1.0 / (std::sqrt(2.0 * M_PI) * sigma);
    const double z = (x - mu) / sigma;
    return c * std::exp(-0.5 * z * z);
}

static Eigen::VectorXd build_cyclic_kernel(int N, double dx, double delta_m, double sigma_m) {
    double mu_cells = delta_m / dx;
    double sigma_cells = sigma_m / dx;

    Eigen::VectorXd k(N);
    int half = N / 2;
    for (int idx = 0; idx < N; ++idx) {
        int d = idx;
        if (idx >= half) d = idx - N;
        double v = std::exp(-0.5 * std::pow((d - mu_cells) / sigma_cells, 2.0));
        k(idx) = v;
    }
    k /= k.sum();
    return k;
}

static Eigen::VectorXd predict_cyclic_direct(const Eigen::VectorXd& bel_prev, const Eigen::VectorXd& kernel) {
    int N = static_cast<int>(bel_prev.size());
    Eigen::VectorXd bel_bar(N);
    bel_bar.setZero();

    for (int j = 0; j < N; ++j) {
        double s = 0.0;
        for (int i = 0; i < N; ++i) {
            int k = (j - i) % N;
            if (k < 0) k += N;
            s += kernel(k) * bel_prev(i);
        }
        bel_bar(j) = s;
    }

    // numerical hygiene
    for (int j = 0; j < N; ++j) bel_bar(j) = std::max(0.0, bel_bar(j));
    bel_bar /= bel_bar.sum();
    return bel_bar;
}

static Eigen::VectorXd likelihood_vector(int N, double dx, double landmark_x, double z_meas, double sigma_z, double half_period) {
    Eigen::VectorXd l(N);
    for (int i = 0; i < N; ++i) {
        double x = i * dx;
        double d = wrap_interval(x - landmark_x, half_period);
        l(i) = std::max(gaussian_pdf(z_meas, d, sigma_z), 1e-300);
    }
    return l;
}

static Eigen::VectorXd bayes_step(const Eigen::VectorXd& bel_prev,
                                 const Eigen::VectorXd& kernel,
                                 double z_meas, double sigma_z,
                                 double landmark_x, double dx, double half_period) {
    Eigen::VectorXd bel_bar = predict_cyclic_direct(bel_prev, kernel);
    Eigen::VectorXd l = likelihood_vector(static_cast<int>(bel_prev.size()), dx, landmark_x, z_meas, sigma_z, half_period);
    Eigen::VectorXd bel = bel_bar.array() * l.array();
    bel /= bel.sum();
    return bel;
}

int main() {
    // Discretization
    const double L = 10.0;
    const int    N = 200;
    const double dx = L / N;
    const double half_period = L / 2.0;

    // Models
    const double u_delta_m = 0.35;
    const double sigma_m   = 0.12;
    const double sigma_z   = 0.20;
    const double landmark_x = 2.0;

    Eigen::VectorXd kernel = build_cyclic_kernel(N, dx, u_delta_m, sigma_m);

    // Initial belief: uniform
    Eigen::VectorXd bel = Eigen::VectorXd::Ones(N) / static_cast<double>(N);

    // RNG
    std::mt19937 gen(7);
    std::normal_distribution<double> n_m(0.0, sigma_m);
    std::normal_distribution<double> n_z(0.0, sigma_z);

    double x_true = 7.0;

    const int T = 25;
    std::cout << "t, x_true[m], x_hat_MAP[m], z_meas[m]\n";
    for (int t = 1; t <= T; ++t) {
        x_true = std::fmod(x_true + u_delta_m + n_m(gen), L);
        if (x_true < 0.0) x_true += L;

        double z_true = wrap_interval(x_true - landmark_x, half_period);
        double z_meas = wrap_interval(z_true + n_z(gen), half_period);

        bel = bayes_step(bel, kernel, z_meas, sigma_z, landmark_x, dx, half_period);

        Eigen::Index idx_hat = 0;
        bel.maxCoeff(&idx_hat);
        double x_hat = static_cast<double>(idx_hat) * dx;

        std::cout << t << ", " << x_true << ", " << x_hat << ", " << z_meas << "\n";
    }

    return 0;
}
