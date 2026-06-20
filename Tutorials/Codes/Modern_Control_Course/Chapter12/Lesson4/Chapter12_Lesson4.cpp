// Chapter12_Lesson4.cpp
// Modern Control — Chapter 12, Lesson 4
// Finite-horizon controllability Gramian and energy directions
// Requires Eigen, including unsupported MatrixFunctions module.
// Example compile command:
// g++ -std=c++17 Chapter12_Lesson4.cpp -I /path/to/eigen -O2 -o Chapter12_Lesson4

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

Matrix finiteHorizonGramian(const Matrix& A, const Matrix& B, double T, int steps) {
    const int n = A.rows();
    Matrix W = Matrix::Zero(n, n);
    const double ds = T / static_cast<double>(steps);

    for (int k = 0; k <= steps; ++k) {
        const double s = k * ds;
        Matrix Phi = (A * s).exp();
        Matrix integrand = Phi * B * B.transpose() * Phi.transpose();
        double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
        W += weight * integrand * ds;
    }
    return 0.5 * (W + W.transpose());
}

double minimumEnergy(const Matrix& W, const Vector& delta) {
    Vector alpha = W.ldlt().solve(delta);
    return delta.dot(alpha);
}

int main() {
    Matrix A(2, 2);
    A << -1.0, 0.0,
          0.0, -4.0;

    Matrix B(2, 1);
    B << 1.0,
         0.08;

    const double T = 2.0;
    Matrix W = finiteHorizonGramian(A, B, T, 4000);

    Eigen::SelfAdjointEigenSolver<Matrix> solver(W);
    Vector lambda = solver.eigenvalues();
    Matrix Q = solver.eigenvectors();

    std::cout << std::setprecision(10);
    std::cout << "Wc(T) =\n" << W << "\n\n";
    std::cout << "Eigenvalues = " << lambda.transpose() << "\n";
    std::cout << "Condition number = " << lambda(lambda.size() - 1) / lambda(0) << "\n\n";
    std::cout << "Eigenvectors (columns) =\n" << Q << "\n\n";

    for (int i = 0; i < 2; ++i) {
        Vector q = Q.col(i);
        std::cout << "Energy to reach q_" << (i + 1) << " = "
                  << minimumEnergy(W, q) << " ; expected = " << 1.0 / lambda(i) << "\n";
    }

    Vector e1(2), e2(2);
    e1 << 1.0, 0.0;
    e2 << 0.0, 1.0;
    std::cout << "\nEnergy to reach e1 = " << minimumEnergy(W, e1) << "\n";
    std::cout << "Energy to reach e2 = " << minimumEnergy(W, e2) << "\n";
    return 0;
}
