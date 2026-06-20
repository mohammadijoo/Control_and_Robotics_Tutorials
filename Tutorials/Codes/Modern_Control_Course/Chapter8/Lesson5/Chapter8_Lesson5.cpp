/*
Chapter8_Lesson5.cpp

Dense computation of Phi(t)=exp(A t) in C++ using Eigen.

Requires Eigen with the unsupported MatrixFunctions module:
  g++ -std=c++17 Chapter8_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter8_Lesson5

The unsupported module provides matrix exponential through:
  #include <unsupported/Eigen/MatrixFunctions>
*/

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;

MatrixXd expm_taylor_scaling_squaring(const MatrixXd& M, int order = 40) {
    const int n = static_cast<int>(M.rows());
    double inf_norm = M.cwiseAbs().rowwise().sum().maxCoeff();
    int s = 0;
    if (inf_norm > 0.0) {
        s = std::max(0, static_cast<int>(std::ceil(std::log2(inf_norm))));
    }

    MatrixXd A_scaled = M / std::pow(2.0, s);
    MatrixXd E = MatrixXd::Identity(n, n);
    MatrixXd term = MatrixXd::Identity(n, n);

    for (int k = 1; k <= order; ++k) {
        term = (term * A_scaled) / static_cast<double>(k);
        E += term;
    }

    for (int i = 0; i < s; ++i) {
        E = E * E;
    }

    return E;
}

int main() {
    MatrixXd A(3, 3);
    A << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
        -2.0,-3.0,-4.0;

    MatrixXd B(3, 1);
    B << 0.0, 0.0, 1.0;

    double t = 0.5;
    double h = 0.1;

    MatrixXd Phi_eigen = (A * t).exp();
    MatrixXd Phi_scratch = expm_taylor_scaling_squaring(A * t, 45);

    std::cout << "A =\n" << A << "\n\n";

    std::cout << "Phi(t) from Eigen MatrixFunctions:\n" << Phi_eigen << "\n\n";
    std::cout << "Phi(t) from scratch scaling-and-squaring Taylor:\n" << Phi_scratch << "\n\n";
    std::cout << "Infinity-norm difference:\n"
              << (Phi_eigen - Phi_scratch).cwiseAbs().rowwise().sum().maxCoeff()
              << "\n\n";

    double s = 0.25;
    MatrixXd semigroup =
        (A * t).exp() * (A * s).exp() - (A * (t + s)).exp();

    std::cout << "Semigroup error ||Phi(t)Phi(s)-Phi(t+s)||_inf:\n"
              << semigroup.cwiseAbs().rowwise().sum().maxCoeff() << "\n\n";

    MatrixXd inverse_error =
        (A * t).exp() * (-A * t).exp() - MatrixXd::Identity(3, 3);

    std::cout << "Inverse error ||Phi(t)Phi(-t)-I||_inf:\n"
              << inverse_error.cwiseAbs().rowwise().sum().maxCoeff() << "\n\n";

    // Van Loan block exponential for exact zero-order-hold discretization.
    MatrixXd M = MatrixXd::Zero(4, 4);
    M.block(0, 0, 3, 3) = A;
    M.block(0, 3, 3, 1) = B;

    MatrixXd EM = (M * h).exp();
    MatrixXd Ad = EM.block(0, 0, 3, 3);
    MatrixXd Bd = EM.block(0, 3, 3, 1);

    std::cout << "Exact ZOH discretization with h=0.1\n";
    std::cout << "Ad =\n" << Ad << "\n\n";
    std::cout << "Bd =\n" << Bd << "\n";

    return 0;
}
