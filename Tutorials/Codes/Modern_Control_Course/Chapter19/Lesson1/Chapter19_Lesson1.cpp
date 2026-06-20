/*
Chapter19_Lesson1.cpp
Modern Control - Chapter 19, Lesson 1
Controllable/Uncontrollable Subspaces using Eigen.

Compile example:
    g++ -std=c++17 Chapter19_Lesson1.cpp -I /path/to/eigen -O2 -o Chapter19_Lesson1
*/

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <limits>
#include <algorithm>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    MatrixXd Wc(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Wc.block(0, k * m, n, m) = Ak * B;
        Ak = Ak * A;
    }
    return Wc;
}

int numericalRankFromSingularValues(const VectorXd& s, double tol) {
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            ++r;
        }
    }
    return r;
}

int main() {
    MatrixXd A(3, 3);
    A << 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, -2.0;

    MatrixXd B(3, 1);
    B << 0.0,
         1.0,
         0.0;

    MatrixXd Wc = controllabilityMatrix(A, B);

    Eigen::JacobiSVD<MatrixXd> svd(Wc, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    double sigmaMax = (s.size() > 0) ? s(0) : 1.0;
    double eps = std::numeric_limits<double>::epsilon();
    double tol = std::max(Wc.rows(), Wc.cols()) * eps * sigmaMax;
    int r = numericalRankFromSingularValues(s, tol);
    int n = static_cast<int>(A.rows());

    MatrixXd U = svd.matrixU();
    MatrixXd Qc = U.leftCols(r);
    MatrixXd Qu = U.rightCols(n - r);
    MatrixXd T(n, n);
    T << Qc, Qu;

    MatrixXd Abar = T.transpose() * A * T;
    MatrixXd Bbar = T.transpose() * B;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "A =\n" << A << "\n\n";
    std::cout << "B =\n" << B << "\n\n";
    std::cout << "Wc = [B AB ... A^(n-1)B] =\n" << Wc << "\n\n";
    std::cout << "Singular values = " << s.transpose() << "\n";
    std::cout << "Rank = " << r << " out of n = " << n << "\n";
    std::cout << ((r == n) ? "System is controllable.\n\n" : "System is not controllable.\n\n");

    std::cout << "Qc basis =\n" << Qc << "\n\n";
    std::cout << "Qu complement basis =\n" << Qu << "\n\n";
    std::cout << "T = [Qc Qu] =\n" << T << "\n\n";
    std::cout << "Abar = T^T A T =\n" << Abar << "\n\n";
    std::cout << "Bbar = T^T B =\n" << Bbar << "\n\n";

    if (r < n) {
        std::cout << "Abar lower-left block =\n" << Abar.block(r, 0, n - r, r) << "\n\n";
        std::cout << "Bbar lower block =\n" << Bbar.block(r, 0, n - r, B.cols()) << "\n";
    }

    return 0;
}
