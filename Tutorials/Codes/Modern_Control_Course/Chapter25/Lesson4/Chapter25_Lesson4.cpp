/*
Chapter25_Lesson4.cpp
Impact of Model Uncertainty on State-Feedback Designs

Compile example with Eigen installed:
    g++ Chapter25_Lesson4.cpp -std=c++17 -O2 -I /path/to/eigen -o Chapter25_Lesson4

This example uses a 2x2 SISO plant and Ackermann's formula from scratch.
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <random>

using Matrix2 = Eigen::Matrix2d;
using Vector2 = Eigen::Vector2d;
using Row2 = Eigen::RowVector2d;

Row2 ackermann2(const Matrix2& A, const Vector2& B, double p1, double p2) {
    Eigen::Matrix2d C;
    C.col(0) = B;
    C.col(1) = A * B;

    if (std::abs(C.determinant()) < 1e-12) {
        throw std::runtime_error("System is not controllable or is ill-conditioned.");
    }

    // Desired characteristic polynomial: s^2 + a1 s + a0
    double a1 = -(p1 + p2);
    double a0 = p1 * p2;
    Matrix2 phiA = A * A + a1 * A + a0 * Matrix2::Identity();

    Row2 en;
    en << 0.0, 1.0;
    return en * C.inverse() * phiA;
}

int main() {
    Matrix2 A;
    A << 0.0, 1.0,
        -2.0, -0.5;
    Vector2 B;
    B << 0.0, 1.0;

    Row2 K = ackermann2(A, B, -2.0, -3.0);
    Matrix2 Acl = A - B * K;

    std::cout << "K = " << K << "\n";
    std::cout << "Nominal closed-loop eigenvalues:\n"
              << Eigen::EigenSolver<Matrix2>(Acl).eigenvalues() << "\n\n";

    std::mt19937 gen(25);
    std::normal_distribution<double> normal(0.0, 0.05);
    std::uniform_real_distribution<double> uniform(-0.25, 0.25);

    int N = 5000;
    int unstable = 0;
    double worst = -1e9;

    for (int i = 0; i < N; ++i) {
        Matrix2 dA;
        dA << normal(gen), normal(gen), normal(gen), normal(gen);
        double rho = uniform(gen);
        Vector2 Btrue = (1.0 + rho) * B;
        Matrix2 Atrue = A + dA;
        Matrix2 Acl_true = Atrue - Btrue * K;
        auto eigs = Eigen::EigenSolver<Matrix2>(Acl_true).eigenvalues();
        double maxReal = std::max(eigs(0).real(), eigs(1).real());
        worst = std::max(worst, maxReal);
        if (maxReal >= 0.0) unstable++;
    }

    std::cout << "Unstable samples = " << unstable << " out of " << N << "\n";
    std::cout << "Worst observed max real part = " << worst << "\n";
    return 0;
}
