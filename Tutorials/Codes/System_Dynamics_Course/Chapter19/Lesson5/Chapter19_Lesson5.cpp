// Chapter19_Lesson5.cpp
// Discretized 1D heat equation + discrete-time Riccati iteration (DARE) using Eigen
//
// Build with (example):
//   g++ -O2 -std=c++17 Chapter19_Lesson5.cpp -I /path/to/eigen -o heat_lqr

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static MatrixXd laplacian_dirichlet(int N, double L) {
    double dx = L / (N + 1.0);
    MatrixXd D2 = MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) {
        D2(i, i) = -2.0;
        if (i - 1 >= 0) D2(i, i - 1) = 1.0;
        if (i + 1 <  N) D2(i, i + 1) = 1.0;
    }
    return D2 / (dx * dx);
}

static VectorXd grid_interior(int N, double L) {
    double dx = L / (N + 1.0);
    VectorXd x(N);
    for (int i = 0; i < N; ++i) x(i) = dx * (i + 1.0);
    return x;
}

static VectorXd gaussian_shape(const VectorXd& x, double x0, double sigma) {
    VectorXd b(x.size());
    for (int i = 0; i < x.size(); ++i) {
        double z = (x(i) - x0) / sigma;
        b(i) = std::exp(-0.5 * z * z);
    }
    return b;
}

// Simple DARE fixed-point iteration:
//   P_{k+1} = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A
static MatrixXd dare_iterate(const MatrixXd& A, const MatrixXd& B,
                             const MatrixXd& Q, const MatrixXd& R,
                             int iters = 5000, double tol = 1e-10) {
    MatrixXd P = Q;
    for (int k = 0; k < iters; ++k) {
        MatrixXd BtPB = B.transpose() * P * B;
        MatrixXd S = R + BtPB;
        MatrixXd K = S.inverse() * (B.transpose() * P * A); // (m x n)
        MatrixXd Pnext = Q + A.transpose() * P * A - A.transpose() * P * B * K;
        double err = (Pnext - P).norm() / (P.norm() + 1e-12);
        P = Pnext;
        if (err < tol) break;
    }
    return P;
}

int main() {
    // Semi-discrete model: x_dot = A x + B u  -> Euler: x_{k+1} = Ad x_k + Bd u_k
    const double alpha = 0.12;
    const double L = 1.0;
    const int N = 60;

    MatrixXd D2 = laplacian_dirichlet(N, L);
    MatrixXd A = alpha * D2;

    VectorXd xgrid = grid_interior(N, L);
    VectorXd b = gaussian_shape(xgrid, 0.25, 0.07);
    b /= b.norm();
    MatrixXd B = b;                 // (N x 1)

    // Discretize
    const double dt = 2e-3;
    MatrixXd Ad = MatrixXd::Identity(N, N) + dt * A;
    MatrixXd Bd = dt * B;

    MatrixXd Q = MatrixXd::Identity(N, N);
    MatrixXd R(1,1); R(0,0) = 2e-3;

    MatrixXd P = dare_iterate(Ad, Bd, Q, R, 20000, 1e-12);

    MatrixXd S = R + Bd.transpose() * P * Bd;
    MatrixXd K = S.inverse() * (Bd.transpose() * P * Ad); // (1 x N)

    // Initial condition
    VectorXd x0(N);
    for (int i = 0; i < N; ++i) {
        double z = xgrid(i) - 0.75;
        x0(i) = std::exp(-80.0 * z * z);
    }

    // Simulate
    int steps = int(4.0 / dt);
    VectorXd x = x0;
    double E0 = 0.5 * x.squaredNorm();
    for (int k = 0; k < steps; ++k) {
        double u = -(K * x)(0,0);
        x = Ad * x + Bd * u;
    }
    double E1 = 0.5 * x.squaredNorm();

    std::cout << "N=" << N << " dt=" << dt << "\n";
    std::cout << "Energy E(0)=" << E0 << "  E(end)=" << E1
              << "  decay=" << (E1/(E0+1e-18)) << "\n";
    return 0;
}
