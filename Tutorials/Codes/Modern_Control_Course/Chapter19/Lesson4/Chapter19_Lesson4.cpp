// Chapter19_Lesson4.cpp
// Identification of a minimal realization via sequential Kalman decomposition.
// Requires Eigen 3: g++ -std=c++17 Chapter19_Lesson4.cpp -I /path/to/eigen -O2

#include <Eigen/Dense>
#include <complex>
#include <iostream>
#include <vector>

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd W(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        W.block(0, k * m, n, m) = Ak * B;
        Ak = Ak * A;
    }
    return W;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = A.rows();
    const int p = C.rows();
    MatrixXd W(n * p, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        W.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return W;
}

int svdRank(const MatrixXd& M, double tol = 1e-10) {
    JacobiSVD<MatrixXd> svd(M);
    const VectorXd s = svd.singularValues();
    if (s.size() == 0) return 0;
    const double threshold = tol * std::max(M.rows(), M.cols()) * std::max(s(0), 1.0);
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > threshold) ++r;
    }
    return r;
}

struct MinimalResult {
    MatrixXd A, B, C, D;
    int reachableRank;
    int observableRankAfterReachable;
};

MinimalResult kalmanMinimalRealization(const MatrixXd& A, const MatrixXd& B,
                                       const MatrixXd& C, const MatrixXd& D) {
    const int n = A.rows();

    MatrixXd Wc = controllabilityMatrix(A, B);
    JacobiSVD<MatrixXd> svdc(Wc, Eigen::ComputeFullU | Eigen::ComputeFullV);
    MatrixXd Tc = svdc.matrixU();
    int rc = svdRank(Wc);

    MatrixXd Ac = Tc.transpose() * A * Tc;
    MatrixXd Bc = Tc.transpose() * B;
    MatrixXd Cc = C * Tc;

    MatrixXd Ar = Ac.block(0, 0, rc, rc);
    MatrixXd Br = Bc.block(0, 0, rc, B.cols());
    MatrixXd Cr = Cc.block(0, 0, C.rows(), rc);

    MatrixXd Wo = observabilityMatrix(Ar, Cr);
    JacobiSVD<MatrixXd> svdo(Wo, Eigen::ComputeFullU | Eigen::ComputeFullV);
    MatrixXd V = svdo.matrixV();
    int ro = svdRank(Wo);

    MatrixXd To(rc, rc);
    if (ro < rc) {
        To << V.rightCols(rc - ro), V.leftCols(ro);
    } else {
        To = V.leftCols(ro);
    }

    MatrixXd Ao = To.transpose() * Ar * To;
    MatrixXd Bo = To.transpose() * Br;
    MatrixXd Co = Cr * To;

    int nUnobs = rc - ro;
    MinimalResult result;
    result.A = Ao.block(nUnobs, nUnobs, ro, ro);
    result.B = Bo.block(nUnobs, 0, ro, B.cols());
    result.C = Co.block(0, nUnobs, C.rows(), ro);
    result.D = D;
    result.reachableRank = rc;
    result.observableRankAfterReachable = ro;
    return result;
}

std::complex<double> transferValue(const MatrixXd& A, const MatrixXd& B,
                                   const MatrixXd& C, const MatrixXd& D,
                                   std::complex<double> s) {
    using MatrixXcd = Eigen::MatrixXcd;
    const int n = A.rows();
    MatrixXcd Ac = A.cast<std::complex<double>>();
    MatrixXcd Bc = B.cast<std::complex<double>>();
    MatrixXcd Cc = C.cast<std::complex<double>>();
    MatrixXcd Dc = D.cast<std::complex<double>>();
    MatrixXcd SIminusA = s * MatrixXcd::Identity(n, n) - Ac;
    MatrixXcd G = Cc * SIminusA.fullPivLu().solve(Bc) + Dc;
    return G(0, 0);
}

int main() {
    MatrixXd A(4, 4);
    A << 0.0, 1.0, 0.0, 0.0,
        -2.0, -3.0, 0.0, 0.0,
         0.0, 0.0, -4.0, 0.0,
         0.0, 0.0, 0.0, -5.0;

    MatrixXd B(4, 1);
    B << 0.0, 1.0, 1.0, 0.0;

    MatrixXd C(1, 4);
    C << 1.0, 0.0, 0.0, 2.0;

    MatrixXd D(1, 1);
    D << 0.0;

    MinimalResult minsys = kalmanMinimalRealization(A, B, C, D);

    std::cout << "rank controllability = " << minsys.reachableRank << "\n";
    std::cout << "rank observability after reachable reduction = "
              << minsys.observableRankAfterReachable << "\n";
    std::cout << "Amin:\n" << minsys.A << "\n";
    std::cout << "Bmin:\n" << minsys.B << "\n";
    std::cout << "Cmin:\n" << minsys.C << "\n";

    std::vector<std::complex<double>> testPoints = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {1.0, 2.0}};
    for (const auto& s : testPoints) {
        auto gf = transferValue(A, B, C, D, s);
        auto gm = transferValue(minsys.A, minsys.B, minsys.C, minsys.D, s);
        std::cout << "s = " << s << ", Gfull = " << gf
                  << ", Gmin = " << gm
                  << ", error = " << std::abs(gf - gm) << "\n";
    }

    return 0;
}
