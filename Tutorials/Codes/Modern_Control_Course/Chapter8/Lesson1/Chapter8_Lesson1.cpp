// Chapter8_Lesson1.cpp
// Modern Control — Chapter 8, Lesson 1: Definition of the State Transition Matrix
//
// This example computes Phi(t,t0) ≈ exp(A*(t-t0)) using a truncated power series.
// For production, prefer a robust library implementation (e.g., Eigen's MatrixFunctions).
//
// Dependencies (recommended): Eigen (header-only)
// Compile (example):
//   g++ -O2 -std=c++17 Chapter8_Lesson1.cpp -I /path/to/eigen -o Chapter8_Lesson1
//
// Run:
//   ./Chapter8_Lesson1

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

static Eigen::MatrixXd expm_series(const Eigen::MatrixXd& A, double tau, int terms) {
    const int n = static_cast<int>(A.rows());
    Eigen::MatrixXd M = A * tau;
    Eigen::MatrixXd term = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd sum  = Eigen::MatrixXd::Identity(n, n);

    for (int k = 1; k <= terms; ++k) {
        term = (term * M) / static_cast<double>(k);
        sum  = sum + term;
    }
    return sum;
}

int main() {
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
        -2.0, -3.0;

    Eigen::Vector2d x0;
    x0 << 1.0, -0.5;

    double t0 = 0.0;
    double t  = 1.25;
    double tau = t - t0;

    // Series truncation (increase for higher accuracy)
    int terms = 30;
    Eigen::Matrix2d Phi = expm_series(A, tau, terms);

    Eigen::Vector2d x = Phi * x0;

    std::cout << std::setprecision(12);
    std::cout << "A =\n" << A << "\n\n";
    std::cout << "tau = t - t0 = " << tau << "\n\n";
    std::cout << "Phi(t,t0) approx (series, terms=" << terms << ") =\n" << Phi << "\n\n";
    std::cout << "x(t) = Phi x0 =\n" << x << "\n";

    // Consistency check: finite-difference derivative of Phi vs A*Phi
    double dt = 1e-6;
    Eigen::Matrix2d Phi_t   = expm_series(A, tau, terms);
    Eigen::Matrix2d Phi_tdt = expm_series(A, tau + dt, terms);
    Eigen::Matrix2d Phi_dot_fd = (Phi_tdt - Phi_t) / dt;
    Eigen::Matrix2d residual = Phi_dot_fd - (A * Phi_t);
    std::cout << "\n||Phi_dot_fd - A Phi||_F = " << residual.norm() << "\n";

    return 0;
}
