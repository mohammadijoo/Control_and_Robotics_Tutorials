/*
Chapter24_Lesson5.cpp
Modern Control - Chapter 24, Lesson 5
Design examples for multi-input pole placement using Eigen.

Dependency:
    Eigen 3.x
Compile example:
    g++ -std=c++17 Chapter24_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter24_Lesson5
*/

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <random>
#include <vector>

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd C(n, n * m);
    MatrixXd Ap = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        if (k > 0) Ap = Ap * A;
        C.block(0, k * m, n, m) = Ap * B;
    }
    return C;
}

int numericalRank(const MatrixXd& M, double tol = 1e-9) {
    JacobiSVD<MatrixXd> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) ++r;
    }
    return r;
}

MatrixXd nullspace(const MatrixXd& M, double tol = 1e-9) {
    JacobiSVD<MatrixXd> svd(M, Eigen::ComputeFullV);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) ++rank;
    }
    return svd.matrixV().rightCols(M.cols() - rank);
}

MatrixXd eigenstructurePolePlacement(
    const MatrixXd& A,
    const MatrixXd& B,
    const std::vector<double>& poles
) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd V = MatrixXd::Zero(n, n);
    MatrixXd F = MatrixXd::Zero(m, n);

    std::mt19937 gen(24);
    std::normal_distribution<double> normal(0.0, 1.0);

    for (int i = 0; i < n; ++i) {
        double lambda = poles[i];
        MatrixXd M(n, n + m);
        M.block(0, 0, n, n) = A - lambda * MatrixXd::Identity(n, n);
        M.block(0, n, n, m) = -B;

        MatrixXd N = nullspace(M);
        if (N.cols() == 0) {
            throw std::runtime_error("Empty nullspace for a requested pole.");
        }

        VectorXd bestV(n), bestF(m);
        bool accepted = false;
        for (int trial = 0; trial < 300; ++trial) {
            VectorXd q(N.cols());
            if (trial == 0) {
                q.setOnes();
            } else {
                for (int j = 0; j < q.size(); ++j) q(j) = normal(gen);
            }
            VectorXd s = N * q;
            VectorXd v = s.head(n);
            VectorXd f = s.tail(m);
            MatrixXd Vtrial = V;
            Vtrial.col(i) = v;
            if (numericalRank(Vtrial.leftCols(i + 1)) == i + 1) {
                bestV = v;
                bestF = f;
                accepted = true;
                break;
            }
        }
        if (!accepted) {
            throw std::runtime_error("Could not select independent eigenvector.");
        }
        V.col(i) = bestV;
        F.col(i) = bestF;
    }

    if (numericalRank(V) < n) {
        throw std::runtime_error("Final eigenvector matrix is singular.");
    }
    return F * V.inverse();
}

int main() {
    MatrixXd A(4, 4);
    A << 0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1,
        -2,-5,-4,-1;

    MatrixXd B(4, 2);
    B << 0, 0,
         1, 0,
         0, 0,
         0, 1;

    std::vector<double> poles = {-1.0, -2.0, -3.0, -4.0};

    MatrixXd C = controllabilityMatrix(A, B);
    std::cout << "rank(C) = " << numericalRank(C) << " out of " << A.rows() << "\n\n";

    MatrixXd K = eigenstructurePolePlacement(A, B, poles);
    std::cout << "K =\n" << K << "\n\n";

    Eigen::EigenSolver<MatrixXd> es(A - B * K);
    std::cout << "Closed-loop eigenvalues:\n" << es.eigenvalues() << "\n";
    return 0;
}
