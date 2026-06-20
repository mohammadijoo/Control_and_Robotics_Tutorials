/*
Chapter24_Lesson1.cpp
Existence conditions for MIMO pole assignment using Eigen.

Build example:
  g++ -std=c++17 Chapter24_Lesson1.cpp -I /path/to/eigen -O2 -o Chapter24_Lesson1
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <vector>

using Matrix = Eigen::MatrixXd;
using CMatrix = Eigen::MatrixXcd;

int numericalRank(const Matrix& M, double tol = 1e-10) {
    Eigen::JacobiSVD<Matrix> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) {
            ++r;
        }
    }
    return r;
}

int numericalRankComplex(const CMatrix& M, double tol = 1e-10) {
    Eigen::JacobiSVD<CMatrix> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) {
            ++r;
        }
    }
    return r;
}

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    Matrix C(n, n * m);
    Matrix Apow = Matrix::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.block(0, k * m, n, m) = Apow * B;
        Apow = A * Apow;
    }
    return C;
}

int pbhRank(const Matrix& A, const Matrix& B, std::complex<double> lambda, double tol = 1e-10) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    CMatrix M(n, n + m);
    M.leftCols(n) = lambda * CMatrix::Identity(n, n) - A.cast<std::complex<double>>();
    M.rightCols(m) = B.cast<std::complex<double>>();
    return numericalRankComplex(M, tol);
}

bool isControllableKalman(const Matrix& A, const Matrix& B) {
    return numericalRank(controllabilityMatrix(A, B)) == A.rows();
}

bool isControllablePBH(const Matrix& A, const Matrix& B) {
    const int n = static_cast<int>(A.rows());
    Eigen::EigenSolver<Matrix> es(A);
    for (int i = 0; i < n; ++i) {
        if (pbhRank(A, B, es.eigenvalues()(i)) != n) {
            return false;
        }
    }
    return true;
}

bool isStabilizable(const Matrix& A, const Matrix& B, double tol = 1e-10) {
    const int n = static_cast<int>(A.rows());
    Eigen::EigenSolver<Matrix> es(A);
    for (int i = 0; i < n; ++i) {
        std::complex<double> lam = es.eigenvalues()(i);
        if (lam.real() >= -tol && pbhRank(A, B, lam, tol) != n) {
            return false;
        }
    }
    return true;
}

int main() {
    Matrix A(3, 3);
    A << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
        -1.0, -5.0, -6.0;

    Matrix B(3, 2);
    B << 0.0, 0.0,
         1.0, 0.0,
         0.0, 1.0;

    std::cout << "Controllability matrix rank: "
              << numericalRank(controllabilityMatrix(A, B)) << "\n";
    std::cout << "Kalman controllable: " << std::boolalpha
              << isControllableKalman(A, B) << "\n";
    std::cout << "PBH controllable: " << std::boolalpha
              << isControllablePBH(A, B) << "\n";
    std::cout << "Stabilizable: " << std::boolalpha
              << isStabilizable(A, B) << "\n";

    // A valid feedback gain for desired poles {-2, -3, -4}.
    Matrix K(2, 3);
    K << 8.0, 6.0, 1.0,
        -1.0, -5.0, -3.0;

    Matrix Acl = A - B * K;
    Eigen::EigenSolver<Matrix> es(Acl);
    std::cout << "Closed-loop eigenvalues:\n" << es.eigenvalues() << "\n";

    Matrix A_bad = Matrix::Zero(3, 3);
    A_bad(0, 0) = 1.0;
    A_bad(1, 1) = -2.0;
    A_bad(2, 2) = -3.0;
    Matrix B_bad(3, 1);
    B_bad << 0.0, 1.0, 1.0;

    std::cout << "\nNon-controllable example rank: "
              << numericalRank(controllabilityMatrix(A_bad, B_bad)) << "\n";
    std::cout << "Non-controllable example stabilizable: "
              << isStabilizable(A_bad, B_bad) << "\n";
    return 0;
}
