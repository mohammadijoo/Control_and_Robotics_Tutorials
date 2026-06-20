/*
Chapter7_Lesson3.cpp
Modern Control — Chapter 7 (Solutions of LTI State Equations), Lesson 3
Solutions for Constant, Step, and Polynomial Inputs

Dependencies:
  - Eigen (https://eigen.tuxfamily.org/)
  - Eigen unsupported module MatrixFunctions for matrix exponential:
      #include <unsupported/Eigen/MatrixFunctions>

Build example (Linux/macOS, adjust include path):
  g++ -O2 -std=c++17 Chapter7_Lesson3.cpp -I /path/to/eigen -o lesson3

This program computes closed-form state responses for:
  1) constant input u(t)=u0
  2) step input u(t)=u0 * 1(t-ts)
  3) polynomial input u(t)=u0 + u1 t
using matrix exponential and phi-functions computed via truncated series.

Note:
  For large ||A t||, use more robust phi/expm algorithms (scaling-squaring + Padé,
  Krylov, etc.). This code is pedagogical.
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static double factorial(int n) {
    double f = 1.0;
    for (int k = 2; k <= n; ++k) f *= static_cast<double>(k);
    return f;
}

static MatrixXd phi_series(const MatrixXd& Z, int m, int terms = 40) {
    const int n = static_cast<int>(Z.rows());
    MatrixXd I = MatrixXd::Identity(n, n);

    MatrixXd S = I / factorial(m);      // j=0 term
    MatrixXd Zpow = I;                  // Z^0
    for (int j = 1; j < terms; ++j) {
        Zpow = Zpow * Z;                // Z^j
        S += Zpow / factorial(j + m);
    }
    return S;
}

static VectorXd x_constant_input(const MatrixXd& A,
                                 const MatrixXd& B,
                                 const VectorXd& x0,
                                 const VectorXd& u0,
                                 double t) {
    MatrixXd At = A * t;
    MatrixXd E = At.exp();                  // e^{At}
    MatrixXd Phi1 = phi_series(At, 1);
    return (E * x0) + (t * (Phi1 * (B * u0)));
}

static VectorXd x_step_input(const MatrixXd& A,
                             const MatrixXd& B,
                             const VectorXd& x0,
                             const VectorXd& u0,
                             double t,
                             double ts) {
    if (t < ts) {
        return (A * t).exp() * x0;
    } else {
        MatrixXd E = (A * t).exp();
        double dt = t - ts;
        MatrixXd Phi1 = phi_series(A * dt, 1);
        return (E * x0) + (dt * (Phi1 * (B * u0)));
    }
}

static VectorXd x_poly_input(const MatrixXd& A,
                             const MatrixXd& B,
                             const VectorXd& x0,
                             const std::vector<VectorXd>& coeffs,
                             double t) {
    // u(t) = sum_{k=0}^p u_k t^k
    // x(t) = e^{At} x0 + sum_{k=0}^p t^{k+1} k! * phi_{k+1}(At) B u_k
    MatrixXd At = A * t;
    MatrixXd E = At.exp();
    VectorXd x = E * x0;

    for (int k = 0; k < static_cast<int>(coeffs.size()); ++k) {
        MatrixXd Phik1 = phi_series(At, k + 1);
        x += std::pow(t, k + 1) * factorial(k) * (Phik1 * (B * coeffs[k]));
    }
    return x;
}

int main() {
    // Example: 2-state, 1-input system
    MatrixXd A(2,2);
    A << 0.0, 1.0,
        -2.0, -3.0;

    MatrixXd B(2,1);
    B << 0.0,
         1.0;

    VectorXd x0(2);
    x0 << 1.0, 0.0;

    VectorXd u0(1);
    u0 << 2.0;

    VectorXd u1(1);
    u1 << 0.5;

    std::vector<VectorXd> coeffs = {u0, u1};

    double ts = 1.5;

    for (double t : {0.0, 1.0, 2.0, 5.0}) {
        VectorXd xc = x_constant_input(A, B, x0, u0, t);
        VectorXd xs = x_step_input(A, B, x0, u0, t, ts);
        VectorXd xp = x_poly_input(A, B, x0, coeffs, t);

        std::cout << "t = " << t << "\n";
        std::cout << "  x_const = " << xc.transpose() << "\n";
        std::cout << "  x_step  = " << xs.transpose() << "\n";
        std::cout << "  x_poly  = " << xp.transpose() << "\n\n";
    }

    return 0;
}
