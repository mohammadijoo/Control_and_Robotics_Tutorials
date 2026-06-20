/*
Chapter8_Lesson3.cpp

Computing Phi(t) = exp(A t) via eigen-decomposition in C++.

Dependency:
    Eigen 3
Compile example:
    g++ -std=c++17 Chapter8_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter8_Lesson3

This implementation uses Eigen::EigenSolver. It assumes A is diagonalizable.
For production-grade matrix exponentials, compare against matrix-functions
modules or specialized numerical linear algebra libraries.
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <stdexcept>

using Matrix = Eigen::MatrixXd;
using ComplexMatrix = Eigen::MatrixXcd;

ComplexMatrix phiViaEigendecomposition(const Matrix& A, double t, double condTol = 1.0e10) {
    Eigen::EigenSolver<Matrix> solver(A);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen-decomposition failed.");
    }

    ComplexMatrix V = solver.eigenvectors();
    Eigen::VectorXcd lambda = solver.eigenvalues();

    Eigen::FullPivLU<ComplexMatrix> lu(V);
    if (lu.rank() < A.rows()) {
        throw std::runtime_error("A is not diagonalizable: eigenvector matrix is rank deficient.");
    }

    double condEstimate = V.norm() * V.inverse().norm();
    if (condEstimate > condTol) {
        std::cerr << "Warning: eigenvector matrix may be ill-conditioned; estimate = "
                  << condEstimate << std::endl;
    }

    ComplexMatrix expLambdaT = ComplexMatrix::Zero(A.rows(), A.cols());
    for (int i = 0; i < lambda.size(); ++i) {
        expLambdaT(i, i) = std::exp(lambda(i) * t);
    }

    return V * expLambdaT * V.inverse();
}

int main() {
    Matrix A(3, 3);
    A << -1.0,  2.0,  0.0,
          0.0, -2.0,  0.0,
          0.0,  0.0, -0.5;

    double t = 2.0;

    try {
        ComplexMatrix Phi = phiViaEigendecomposition(A, t);

        std::cout << "A =\n" << A << "\n\n";
        std::cout << "Phi(t) via eigen-decomposition =\n" << Phi << "\n\n";

        Eigen::VectorXcd x0(3);
        x0 << 1.0, -1.0, 2.0;
        Eigen::VectorXcd xt = Phi * x0;

        std::cout << "x0 =\n" << x0 << "\n\n";
        std::cout << "x(t) = Phi(t) x0 =\n" << xt << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
