// Chapter20_Lesson4.cpp
// Basic rank-based realization-reduction diagnostics using Eigen.
// Compile example:
//   g++ -std=c++17 Chapter20_Lesson4.cpp -I /path/to/eigen -O2 -o Chapter20_Lesson4

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    MatrixXd R(n, n * B.cols());
    MatrixXd Apow = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        R.block(0, k * B.cols(), n, B.cols()) = Apow * B;
        Apow = Apow * A;
    }
    return R;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = A.rows();
    MatrixXd O(n * C.rows(), n);
    MatrixXd Apow = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * C.rows(), 0, C.rows(), n) = C * Apow;
        Apow = Apow * A;
    }
    return O;
}

int svdRank(const MatrixXd& M, double tol = 1e-10) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    if (s.size() == 0) return 0;
    int r = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) ++r;
    return r;
}

MatrixXd columnSpaceBasis(const MatrixXd& M) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    double tol = 1e-10;
    int r = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) ++r;
    return svd.matrixU().leftCols(r);
}

MatrixXd nullSpaceBasis(const MatrixXd& M) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    double tol = 1e-10;
    int r = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) ++r;
    return svd.matrixV().rightCols(M.cols() - r);
}

int main() {
    MatrixXd A(3, 3);
    A << -1.0, 0.0, 0.0,
          0.0,-2.0, 0.0,
          0.0, 0.0,-3.0;
    MatrixXd B(3, 1);
    B << 1.0, 0.0, 1.0;
    MatrixXd C(1, 3);
    C << 1.0, 0.0, 0.0;
    MatrixXd D(1, 1);
    D << 0.0;

    MatrixXd R = controllabilityMatrix(A, B);
    MatrixXd O = observabilityMatrix(A, C);

    std::cout << "rank(R) = " << svdRank(R) << " of " << A.rows() << "\n";
    std::cout << "rank(O) = " << svdRank(O) << " of " << A.rows() << "\n\n";

    // Stage 1: reachable projection.
    MatrixXd Qr = columnSpaceBasis(R);
    MatrixXd A1 = Qr.transpose() * A * Qr;
    MatrixXd B1 = Qr.transpose() * B;
    MatrixXd C1 = C * Qr;

    // Stage 2: unobservable quotient within the reachable subsystem.
    MatrixXd O1 = observabilityMatrix(A1, C1);
    MatrixXd Qun = nullSpaceBasis(O1);
    int q = Qun.cols();

    MatrixXd Amin, Bmin, Cmin;
    if (q == 0) {
        Amin = A1; Bmin = B1; Cmin = C1;
    } else {
        MatrixXd Qobs = nullSpaceBasis(Qun.transpose());
        MatrixXd T(A1.rows(), A1.rows());
        T << Qun, Qobs;
        MatrixXd Ahat = T.transpose() * A1 * T;
        MatrixXd Bhat = T.transpose() * B1;
        MatrixXd Chat = C1 * T;
        int m = A1.rows() - q;
        Amin = Ahat.block(q, q, m, m);
        Bmin = Bhat.block(q, 0, m, Bhat.cols());
        Cmin = Chat.block(0, q, Chat.rows(), m);
    }

    std::cout << "Amin =\n" << Amin << "\n\n";
    std::cout << "Bmin =\n" << Bmin << "\n\n";
    std::cout << "Cmin =\n" << Cmin << "\n";
    return 0;
}
