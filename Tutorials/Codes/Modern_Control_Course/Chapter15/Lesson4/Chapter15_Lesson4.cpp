// Chapter15_Lesson4.cpp
// Condition Numbers and Sensitivity in State Reconstruction
//
// Build:
//   g++ -std=c++17 -O2 Chapter15_Lesson4.cpp -o Chapter15_Lesson4
//
// This example intentionally uses a small from-scratch 2x2 implementation.
// For larger Modern Control workflows, use Eigen, Armadillo, Blaze, SLICOT,
// or a C++ control library with Lyapunov and state-space routines.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <array>

struct Vec2 {
    double x1;
    double x2;
};

struct Mat2 {
    double a11, a12, a21, a22;
};

double det2(const Mat2& M) {
    return M.a11 * M.a22 - M.a12 * M.a21;
}

Vec2 solve2(const Mat2& M, const Vec2& b) {
    double d = det2(M);
    return {
        ( M.a22 * b.x1 - M.a12 * b.x2) / d,
        (-M.a21 * b.x1 + M.a11 * b.x2) / d
    };
}

std::array<double, 2> eigenvaluesSymmetric2(const Mat2& M) {
    double tr = M.a11 + M.a22;
    double diff = M.a11 - M.a22;
    double rad = std::sqrt(diff * diff + 4.0 * M.a12 * M.a12);
    return {0.5 * (tr - rad), 0.5 * (tr + rad)};
}

int main() {
    const double eps = 0.02;
    const double T = 5.0;
    const int N = 200000;
    const double dt = T / static_cast<double>(N);

    // A = diag(-1, -2), C = [1, eps]
    // True initial condition
    const Vec2 x0_true{1.0, -1.5};

    Mat2 W{0.0, 0.0, 0.0, 0.0};
    Vec2 b{0.0, 0.0};

    for (int k = 0; k < N; ++k) {
        double t = (k + 0.5) * dt;

        double phi1 = std::exp(-t);
        double phi2 = std::exp(-2.0 * t);

        // H(t) = C exp(A t) = [phi1, eps phi2]
        double h1 = phi1;
        double h2 = eps * phi2;

        // Deterministic small measurement disturbance
        double y_clean = h1 * x0_true.x1 + h2 * x0_true.x2;
        double noise = 1.0e-3 * std::sin(37.0 * t);
        double y = y_clean + noise;

        W.a11 += h1 * h1 * dt;
        W.a12 += h1 * h2 * dt;
        W.a21 += h2 * h1 * dt;
        W.a22 += h2 * h2 * dt;

        b.x1 += h1 * y * dt;
        b.x2 += h2 * y * dt;
    }

    Vec2 xhat = solve2(W, b);
    auto evals = eigenvaluesSymmetric2(W);
    double kappa = evals[1] / evals[0];

    std::cout << std::setprecision(12);
    std::cout << "W = [[" << W.a11 << ", " << W.a12 << "], ["
              << W.a21 << ", " << W.a22 << "]]\n";
    std::cout << "lambda_min = " << evals[0] << "\n";
    std::cout << "lambda_max = " << evals[1] << "\n";
    std::cout << "condition number = " << kappa << "\n\n";

    std::cout << "true x0 = [" << x0_true.x1 << ", " << x0_true.x2 << "]\n";
    std::cout << "estimated x0 = [" << xhat.x1 << ", " << xhat.x2 << "]\n";

    // Ridge-regularized reconstruction: (W + alpha I) x = b
    double alpha = 1.0e-5;
    Mat2 Wr{W.a11 + alpha, W.a12, W.a21, W.a22 + alpha};
    Vec2 xr = solve2(Wr, b);
    std::cout << "ridge estimated x0 = [" << xr.x1 << ", " << xr.x2 << "]\n";

    return 0;
}
