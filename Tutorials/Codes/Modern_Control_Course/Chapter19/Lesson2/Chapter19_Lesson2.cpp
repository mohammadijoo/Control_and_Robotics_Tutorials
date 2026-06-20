// Chapter19_Lesson2.cpp
// Observable/unobservable subspaces for an LTI pair (A,C).
// Dependency: Eigen 3 (https://eigen.tuxfamily.org). Compile example:
//   g++ -std=c++17 Chapter19_Lesson2.cpp -I /path/to/eigen -O2 -o Chapter19_Lesson2

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());
    MatrixXd O(p * n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return O;
}

int numericalRank(const MatrixXd& M, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorXd s = svd.singularValues();
    if (tol < 0.0) {
        tol = std::max(M.rows(), M.cols()) * std::numeric_limits<double>::epsilon() * (s.size() ? s(0) : 1.0);
    }
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) { ++r; }
    }
    return r;
}

MatrixXd nullspace(const MatrixXd& M, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    if (tol < 0.0) {
        tol = std::max(M.rows(), M.cols()) * std::numeric_limits<double>::epsilon() * (s.size() ? s(0) : 1.0);
    }
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) { ++r; }
    }
    return svd.matrixV().rightCols(M.cols() - r);
}

MatrixXd rowSpaceBasis(const MatrixXd& M, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    if (tol < 0.0) {
        tol = std::max(M.rows(), M.cols()) * std::numeric_limits<double>::epsilon() * (s.size() ? s(0) : 1.0);
    }
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) { ++r; }
    }
    return svd.matrixV().leftCols(r);
}

int main() {
    MatrixXd A(4, 4);
    A << 0.0, 1.0, 0.0, 0.0,
        -2.0, -3.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, -5.0, -1.0;

    MatrixXd C(1, 4);
    C << 1.0, 0.0, 0.0, 0.0;

    MatrixXd O = observabilityMatrix(A, C);
    MatrixXd Qu = nullspace(O);
    MatrixXd Qo = rowSpaceBasis(O);

    MatrixXd Q(A.rows(), A.cols());
    Q << Qo, Qu;  // For this example, dimensions add to n.

    MatrixXd Abar = Q.transpose() * A * Q;
    MatrixXd Cbar = C * Q;

    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    double invarianceResidual = ((I - Qu * Qu.transpose()) * A * Qu).norm();

    std::cout << std::setprecision(6) << std::fixed;
    std::cout << "Observability matrix O_n:\n" << O << "\n\n";
    std::cout << "rank(O_n) = " << numericalRank(O) << " out of n = " << A.rows() << "\n\n";
    std::cout << "Basis for unobservable subspace ker(O_n):\n" << Qu << "\n\n";
    std::cout << "A-invariance residual = " << invarianceResidual << "\n\n";
    std::cout << "Abar = Q^T A Q:\n" << Abar << "\n\n";
    std::cout << "Cbar = C Q:\n" << Cbar << "\n";

    return 0;
}
