// Chapter19_Lesson3.cpp
// Kalman Decomposition: block structure of A, B, C
// Requires Eigen: https://eigen.tuxfamily.org
//
// Compile example:
//   g++ -std=c++17 Chapter19_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter19_Lesson3
//
// Block order:
//   [ controllable-observable,
//     controllable-unobservable,
//     uncontrollable-observable,
//     uncontrollable-unobservable ]

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

double TOL = 1e-9;

int rankSVD(const MatrixXd& M, double tol = TOL) {
    if (M.size() == 0) return 0;
    Eigen::JacobiSVD<MatrixXd> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) r++;
    }
    return r;
}

MatrixXd nullSpace(const MatrixXd& M, double tol = TOL) {
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    int n = M.cols();
    int r = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) r++;
    int dim = n - r;
    if (dim <= 0) return MatrixXd(M.cols(), 0);
    return svd.matrixV().rightCols(dim);
}

MatrixXd columnSpaceBasis(const MatrixXd& M, double tol = TOL) {
    if (M.cols() == 0) return MatrixXd(M.rows(), 0);
    Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeThinU);
    VectorXd s = svd.singularValues();
    int r = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) r++;
    if (r == 0) return MatrixXd(M.rows(), 0);
    return svd.matrixU().leftCols(r);
}

MatrixXd hstack(const std::vector<MatrixXd>& mats) {
    int rows = mats.empty() ? 0 : mats[0].rows();
    int cols = 0;
    for (const auto& M : mats) cols += M.cols();
    MatrixXd H(rows, cols);
    int c = 0;
    for (const auto& M : mats) {
        H.block(0, c, rows, M.cols()) = M;
        c += M.cols();
    }
    return H;
}

MatrixXd vstack(const std::vector<MatrixXd>& mats) {
    int cols = mats.empty() ? 0 : mats[0].cols();
    int rows = 0;
    for (const auto& M : mats) rows += M.rows();
    MatrixXd V(rows, cols);
    int r = 0;
    for (const auto& M : mats) {
        V.block(r, 0, M.rows(), cols) = M;
        r += M.rows();
    }
    return V;
}

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    int n = A.rows();
    std::vector<MatrixXd> blocks;
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(Ak * B);
        Ak = A * Ak;
    }
    return hstack(blocks);
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    int n = A.rows();
    std::vector<MatrixXd> blocks;
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        blocks.push_back(C * Ak);
        Ak = A * Ak;
    }
    return vstack(blocks);
}

MatrixXd intersectionBasis(const MatrixXd& U, const MatrixXd& V, double tol = TOL) {
    if (U.cols() == 0 || V.cols() == 0) return MatrixXd(U.rows(), 0);
    MatrixXd M(U.rows(), U.cols() + V.cols());
    M << U, -V;
    MatrixXd K = nullSpace(M, tol);
    if (K.cols() == 0) return MatrixXd(U.rows(), 0);
    MatrixXd alpha = K.topRows(U.cols());
    return columnSpaceBasis(U * alpha, tol);
}

MatrixXd appendIndependent(const MatrixXd& current,
                           const MatrixXd& candidates,
                           int targetDim,
                           double tol = TOL) {
    MatrixXd M = current;
    int r = rankSVD(M, tol);
    for (int j = 0; j < candidates.cols(); ++j) {
        MatrixXd trial(M.rows(), M.cols() + 1);
        if (M.cols() > 0) trial.block(0, 0, M.rows(), M.cols()) = M;
        trial.col(M.cols()) = candidates.col(j);
        int rt = rankSVD(trial, tol);
        if (rt > r) {
            M = trial;
            r = rt;
            if (targetDim >= 0 && r >= targetDim) break;
        }
    }
    return M;
}

MatrixXd complementInside(const MatrixXd& container, const MatrixXd& sub, double tol = TOL) {
    MatrixXd completed = appendIndependent(sub, container, container.cols(), tol);
    if (completed.cols() <= sub.cols()) return MatrixXd(container.rows(), 0);
    return completed.rightCols(completed.cols() - sub.cols());
}

struct KalmanResult {
    MatrixXd T;
    MatrixXd Abar;
    MatrixXd Bbar;
    MatrixXd Cbar;
    std::vector<int> dims;
};

KalmanResult kalmanDecomposition(const MatrixXd& A, const MatrixXd& B, const MatrixXd& C) {
    int n = A.rows();

    MatrixXd R = columnSpaceBasis(controllabilityMatrix(A, B));
    MatrixXd N = columnSpaceBasis(nullSpace(observabilityMatrix(A, C)));
    MatrixXd V2 = intersectionBasis(R, N);
    MatrixXd V1 = complementInside(R, V2);
    MatrixXd V4 = complementInside(N, V2);

    MatrixXd V124 = hstack({V1, V2, V4});
    MatrixXd completed = appendIndependent(V124, MatrixXd::Identity(n, n), n);
    MatrixXd V3 = completed.rightCols(completed.cols() - V124.cols());

    MatrixXd T = hstack({V1, V2, V3, V4});
    MatrixXd Tinv = T.inverse();

    KalmanResult out;
    out.T = T;
    out.Abar = Tinv * A * T;
    out.Bbar = Tinv * B;
    out.Cbar = C * T;
    out.dims = {static_cast<int>(V1.cols()), static_cast<int>(V2.cols()),
                static_cast<int>(V3.cols()), static_cast<int>(V4.cols())};
    return out;
}

int main() {
    MatrixXd Ak(4,4);
    Ak << -1.0,  0.0,  0.7,  0.0,
           0.2, -2.0,  0.3, -0.4,
           0.0,  0.0, -3.0,  0.0,
           0.0,  0.0,  0.6, -4.0;

    MatrixXd Bk(4,1);
    Bk << 1.0, 0.5, 0.0, 0.0;

    MatrixXd Ck(1,4);
    Ck << 2.0, 0.0, -1.0, 0.0;

    MatrixXd Tphys(4,4);
    Tphys << 1.0,  0.2,  0.1,  0.0,
             0.0,  1.0, -0.3,  0.4,
             0.2,  0.0,  1.0,  0.1,
             0.1, -0.2,  0.0,  1.0;

    MatrixXd A = Tphys * Ak * Tphys.inverse();
    MatrixXd B = Tphys * Bk;
    MatrixXd C = Ck * Tphys.inverse();

    KalmanResult kr = kalmanDecomposition(A, B, C);

    std::cout << "Block dimensions [co, c_unobs, unctrl_obs, unctrl_unobs]: ";
    for (int d : kr.dims) std::cout << d << " ";
    std::cout << "\n\nAbar:\n" << kr.Abar << "\n\nBbar:\n" << kr.Bbar
              << "\n\nCbar:\n" << kr.Cbar << "\n";

    return 0;
}
