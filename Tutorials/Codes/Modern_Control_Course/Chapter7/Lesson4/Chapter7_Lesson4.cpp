// Chapter7_Lesson4.cpp
/*
Modern Control — Chapter 7, Lesson 4
Output Response Using x(t) and y(t) = Cx(t) + Du(t)

This C++ example uses Eigen to:
1) Compute Phi(t)=exp(A t) using Eigen's MatrixFunctions (unsupported module)
2) Compute analytic response for constant input u(t)=u0 when A is invertible:
       x(t) = exp(A t) x0 + A^{-1}(exp(A t) - I) B u0
       y(t) = C x(t) + D u0
3) Provide an educational forward-Euler simulator as a fallback/validation

Build (example):
  g++ -O2 -std=c++17 Chapter7_Lesson4.cpp -I /path/to/eigen -o Chapter7_Lesson4

Note: Eigen's matrix exponential is in the unsupported module.
*/

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static bool isInvertible(const MatrixXd& A, double tol = 1e-12) {
    Eigen::FullPivLU<MatrixXd> lu(A);
    return lu.isInvertible() && (1.0 / A.fullPivLu().rcond() < 1.0 / tol);
}

int main() {
    // Example: 2-state SISO system
    MatrixXd A(2,2);
    A << 0.0,  1.0,
        -2.0, -3.0;

    MatrixXd B(2,1);
    B << 0.0,
         1.0;

    MatrixXd C(1,2);
    C << 1.0, 0.0;

    MatrixXd D(1,1);
    D << 0.25; // direct feedthrough to show output jump at a step

    VectorXd x0(2);
    x0 << 1.0, 0.0;

    VectorXd u0(1);
    u0 << 1.0;

    const double t_end = 6.0;
    const double h = 0.01;
    const int N = static_cast<int>(std::round(t_end / h)) + 1;

    std::vector<double> t(N);
    for (int k = 0; k < N; ++k) t[k] = k * h;

    // Analytic constant-input response (requires invertible A)
    if (!isInvertible(A)) {
        std::cerr << "A is not invertible; analytic constant-input formula not used.\n";
    } else {
        MatrixXd Ainv = A.inverse();
        std::cout << "Analytic y(t) (constant input) at a few times:\n";
        for (double ti : {0.0, 0.5, 1.0, 2.0, 6.0}) {
            MatrixXd Phi = (A * ti).exp();                    // expm
            VectorXd xt = Phi * x0 + (Ainv * (Phi - MatrixXd::Identity(2,2)) * B) * u0(0);
            VectorXd yt = C * xt + D * u0;
            std::cout << "t=" << ti << ", y=" << yt(0) << "\n";
        }
    }

    // From-scratch Euler simulation
    std::vector<VectorXd> x(N, VectorXd::Zero(2));
    std::vector<VectorXd> y(N, VectorXd::Zero(1));

    x[0] = x0;
    for (int k = 0; k < N-1; ++k) {
        VectorXd uk = u0;
        y[k] = C * x[k] + D * uk;

        VectorXd xdot = A * x[k] + B * uk(0);
        x[k+1] = x[k] + h * xdot;
    }
    y[N-1] = C * x[N-1] + D * u0;

    std::cout << "\nEuler y(t) at a few times:\n";
    for (double ti : {0.0, 0.5, 1.0, 2.0, 6.0}) {
        int k = static_cast<int>(std::round(ti / h));
        std::cout << "t=" << t[k] << ", y=" << y[k](0) << "\n";
    }

    return 0;
}
