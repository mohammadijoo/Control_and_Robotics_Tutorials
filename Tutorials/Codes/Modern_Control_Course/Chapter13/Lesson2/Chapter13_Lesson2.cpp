// Chapter13_Lesson2.cpp
// Observable states and observable subspace for an LTI system.
// Dependency: Eigen (https://eigen.tuxfamily.org). Compile for example with:
// g++ Chapter13_Lesson2.cpp -I /path/to/eigen -O2 -std=c++17 -o Chapter13_Lesson2

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd observabilitySignatureMatrix(const MatrixXd& A, const MatrixXd& C) {
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

int numericalRank(const MatrixXd& M, double tol = 1e-10) {
    Eigen::JacobiSVD<MatrixXd> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) ++r;
    }
    return r;
}

MatrixXd nullSpace(const MatrixXd& M, double tol = 1e-10) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullV);
    const auto& s = svd.singularValues();
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) ++r;
    }
    return svd.matrixV().rightCols(M.cols() - r);
}

MatrixXd rowSpaceBasisAsColumns(const MatrixXd& M, double tol = 1e-10) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullV);
    const auto& s = svd.singularValues();
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) ++r;
    }
    return svd.matrixV().leftCols(r);
}

int main() {
    MatrixXd A(3, 3);
    A << 0.0, 1.0, 0.0,
        -2.0, -3.0, 0.0,
         0.0, 0.0, -4.0;

    MatrixXd C(1, 3);
    C << 1.0, 0.0, 0.0;

    MatrixXd O = observabilitySignatureMatrix(A, C);
    MatrixXd Nu = nullSpace(O);
    MatrixXd Vo = rowSpaceBasisAsColumns(O);

    std::cout << "Observability signature matrix O:\n" << O << "\n\n";
    std::cout << "rank(O) = " << numericalRank(O) << "\n\n";
    std::cout << "Observable subspace basis columns (range of O^T):\n" << Vo << "\n\n";
    std::cout << "Unobservable subspace basis columns (ker O):\n" << Nu << "\n\n";

    VectorXd x0(3);
    x0 << 2.0, -1.0, 5.0;
    MatrixXd Po = Vo * Vo.transpose();
    MatrixXd Pu = Nu.cols() > 0 ? Nu * Nu.transpose() : MatrixXd::Zero(3, 3);

    std::cout << "x0:\n" << x0 << "\n\n";
    std::cout << "observable component:\n" << Po * x0 << "\n\n";
    std::cout << "unobservable component:\n" << Pu * x0 << "\n";
    return 0;
}
