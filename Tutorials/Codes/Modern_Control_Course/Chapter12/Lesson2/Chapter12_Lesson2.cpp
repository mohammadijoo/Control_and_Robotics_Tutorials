// Chapter12_Lesson2.cpp
// Minimum-energy steering for continuous-time LTI systems.
// Requires: Eigen 3 with unsupported MatrixFunctions module.
// Example compile command:
//   g++ -std=c++17 Chapter12_Lesson2.cpp -I /path/to/eigen -O2 -o Chapter12_Lesson2

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityGramian(const MatrixXd& A, const MatrixXd& B, double T, int N = 4001) {
    const int n = A.rows();
    MatrixXd W = MatrixXd::Zero(n, n);
    const double dt = T / static_cast<double>(N - 1);

    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        MatrixXd E = (A * (T - t)).exp();
        MatrixXd F = E * B * B.transpose() * E.transpose();
        double weight = (k == 0 || k == N - 1) ? 0.5 : 1.0;
        W += weight * F;
    }
    return W * dt;
}

VectorXd minimumEnergyInput(
    double t,
    const MatrixXd& A,
    const MatrixXd& B,
    const VectorXd& lambda,
    double T
) {
    MatrixXd E = (A.transpose() * (T - t)).exp();
    return B.transpose() * E * lambda;
}

VectorXd dynamics(
    double t,
    const VectorXd& x,
    const MatrixXd& A,
    const MatrixXd& B,
    const VectorXd& lambda,
    double T
) {
    VectorXd u = minimumEnergyInput(t, A, B, lambda, T);
    return A * x + B * u;
}

int main() {
    MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    MatrixXd B(2, 1);
    B << 0.0,
         1.0;

    double T = 2.0;
    VectorXd x0(2), xf(2);
    x0 << 0.0, 0.0;
    xf << 1.0, 0.0;

    MatrixXd W = controllabilityGramian(A, B, T);
    VectorXd z = (xf - (A * T).exp() * x0);
    VectorXd lambda = W.ldlt().solve(z);
    double Emin = z.dot(lambda);

    std::cout << std::setprecision(10);
    std::cout << "Wc(T):\n" << W << "\n\n";
    std::cout << "lambda:\n" << lambda << "\n\n";
    std::cout << "minimum energy: " << Emin << "\n\n";

    // RK4 simulation of x_dot = A x + B u_star(t)
    int N = 1000;
    double h = T / static_cast<double>(N);
    VectorXd x = x0;
    double t = 0.0;

    for (int k = 0; k < N; ++k) {
        VectorXd k1 = dynamics(t, x, A, B, lambda, T);
        VectorXd k2 = dynamics(t + 0.5*h, x + 0.5*h*k1, A, B, lambda, T);
        VectorXd k3 = dynamics(t + 0.5*h, x + 0.5*h*k2, A, B, lambda, T);
        VectorXd k4 = dynamics(t + h, x + h*k3, A, B, lambda, T);
        x += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
        t += h;
    }

    std::cout << "terminal state reached:\n" << x << "\n\n";
    std::cout << "target state:\n" << xf << "\n\n";
    std::cout << "terminal error norm: " << (x - xf).norm() << "\n";

    return 0;
}
