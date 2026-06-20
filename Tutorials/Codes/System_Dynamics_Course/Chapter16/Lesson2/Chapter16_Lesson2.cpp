// Chapter16_Lesson2.cpp
// Difference Equations and Discrete-Time State-Space Models

#include <iostream>
#include <vector>
#include <array>
#include <iomanip>

std::vector<double> simulateScalarDifference(double a, double b,
                                             const std::vector<double>& u,
                                             double y0) {
    std::vector<double> y(u.size() + 1, 0.0);
    y[0] = y0;
    for (std::size_t k = 0; k < u.size(); ++k) {
        y[k + 1] = a * y[k] + b * u[k];
    }
    return y;
}

struct SimResult {
    std::vector<std::array<double, 2>> X; // states x[k]
    std::vector<double> Y;                // outputs y[k]
};

SimResult simulateDiscreteSS(const std::array<std::array<double, 2>, 2>& A,
                             const std::array<double, 2>& B,
                             const std::array<double, 2>& C,
                             double D,
                             const std::vector<double>& u,
                             std::array<double, 2> x0) {
    SimResult r;
    r.X.resize(u.size() + 1);
    r.Y.resize(u.size());
    r.X[0] = x0;

    auto x = x0;
    for (std::size_t k = 0; k < u.size(); ++k) {
        double uk = u[k];
        double yk = C[0] * x[0] + C[1] * x[1] + D * uk;
        r.Y[k] = yk;

        std::array<double, 2> xnext{};
        xnext[0] = A[0][0] * x[0] + A[0][1] * x[1] + B[0] * uk;
        xnext[1] = A[1][0] * x[0] + A[1][1] * x[1] + B[1] * uk;

        x = xnext;
        r.X[k + 1] = x;
    }
    return r;
}

int main() {
    const int N = 40;

    // Example 1: scalar recursion y[k+1] = a y[k] + b u[k]
    std::vector<double> u1(N, 1.0);
    double a = 0.85, b = 0.2, y0 = 0.0;
    auto y = simulateScalarDifference(a, b, u1, y0);

    std::cout << "Scalar recursion (first 10 samples)\n";
    for (int k = 0; k < 10; ++k) {
        std::cout << "k=" << std::setw(2) << k
                  << "  y=" << std::fixed << std::setprecision(6)
                  << y[k] << "\n";
    }
    std::cout << "\n";

    // Example 2: companion-form state-space for
    // y[k+2] - 1.5 y[k+1] + 0.7 y[k] = u[k]
    std::array<std::array<double, 2>, 2> A{{ {1.5, -0.7}, {1.0, 0.0} }};
    std::array<double, 2> B{{1.0, 0.0}};
    std::array<double, 2> C{{0.0, 1.0}};
    double D = 0.0;

    std::vector<double> u2(N, 0.0);
    for (int k = 0; k < 10; ++k) u2[k] = 1.0;

    std::array<double, 2> x0_ss{{0.0, 0.0}};
    auto result = simulateDiscreteSS(A, B, C, D, u2, x0_ss);

    std::cout << "State-space output (first 10 samples)\n";
    for (int k = 0; k < 10; ++k) {
        std::cout << "k=" << std::setw(2) << k
                  << "  y=" << std::fixed << std::setprecision(6) << result.Y[k]
                  << "  x=[" << result.X[k][0] << ", " << result.X[k][1] << "]\n";
    }

    return 0;
}
