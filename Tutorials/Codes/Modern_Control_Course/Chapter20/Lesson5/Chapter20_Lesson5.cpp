// Chapter20_Lesson5.cpp
/*
Numerical rank diagnostics for realization reduction.

Dependencies:
    Eigen 3
Compile example:
    g++ -std=c++17 Chapter20_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter20_Lesson5
*/

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd matrixPower(const MatrixXd& A, int k) {
    MatrixXd P = MatrixXd::Identity(A.rows(), A.cols());
    for (int i = 0; i < k; ++i) {
        P = P * A;
    }
    return P;
}

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd R(n, n * m);
    MatrixXd Ap = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        R.block(0, k * m, n, m) = Ap * B;
        Ap = Ap * A;
    }
    return R;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = A.rows();
    const int p = C.rows();
    MatrixXd O(n * p, n);
    MatrixXd Ap = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = C * Ap;
        Ap = Ap * A;
    }
    return O;
}

int numericalRank(const MatrixXd& M, double rtol) {
    Eigen::JacobiSVD<MatrixXd> svd(M);
    VectorXd s = svd.singularValues();
    if (s.size() == 0) return 0;
    double threshold = rtol * s(0);
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > threshold) ++r;
    }
    return r;
}

void printSingularValues(const std::string& name, const MatrixXd& M) {
    Eigen::JacobiSVD<MatrixXd> svd(M);
    std::cout << name << " singular values:\n"
              << svd.singularValues().transpose() << "\n\n";
}

int main() {
    MatrixXd A(4, 4);
    A << -0.20,  0.05,  0.00,  0.00,
          0.00, -1.00,  0.10,  0.00,
          0.00,  0.00, -8.00,  0.20,
          0.00,  0.00,  0.00, -20.0;

    MatrixXd B(4, 1);
    B << 1.0, 0.4, 0.05, 0.01;

    MatrixXd C(1, 4);
    C << 1.0, 0.3, 0.02, 0.005;

    MatrixXd Rc = controllabilityMatrix(A, B);
    MatrixXd Ro = observabilityMatrix(A, C);

    double eps = std::numeric_limits<double>::epsilon();
    double rtolC = std::max(Rc.rows(), Rc.cols()) * eps;
    double rtolO = std::max(Ro.rows(), Ro.cols()) * eps;

    printSingularValues("Controllability matrix", Rc);
    printSingularValues("Observability matrix", Ro);

    std::cout << "Numerical controllability rank = "
              << numericalRank(Rc, rtolC) << "\n";
    std::cout << "Numerical observability rank = "
              << numericalRank(Ro, rtolO) << "\n";

    Eigen::JacobiSVD<MatrixXd> svdC(Rc);
    double conditionEstimate = svdC.singularValues()(0) /
                               svdC.singularValues()(svdC.singularValues().size() - 1);

    std::cout << "Estimated condition number of controllability matrix = "
              << conditionEstimate << "\n";

    std::cout << "\nInterpretation:\n";
    std::cout << "A formal full rank result can be misleading when the smallest\n";
    std::cout << "singular values are near the tolerance. In that case, reduction\n";
    std::cout << "or controller synthesis should be based on SVD/QR diagnostics\n";
    std::cout << "rather than exact symbolic rank.\n";

    return 0;
}
