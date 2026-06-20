/*
Chapter28_Lesson5.cpp

Continuous-time LQR preview using Eigen.

This implementation demonstrates Kleinman's policy-iteration method for a
stabilizing continuous-time algebraic Riccati equation (CARE) solution.

Compile example:
    g++ -std=c++17 Chapter28_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter28_Lesson5
*/

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd kron(const MatrixXd& A, const MatrixXd& B) {
    MatrixXd K(A.rows() * B.rows(), A.cols() * B.cols());
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            K.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
        }
    }
    return K;
}

VectorXd vec(const MatrixXd& M) {
    VectorXd v(M.rows() * M.cols());
    int k = 0;
    for (int j = 0; j < M.cols(); ++j) {
        for (int i = 0; i < M.rows(); ++i) {
            v(k++) = M(i, j);
        }
    }
    return v;
}

MatrixXd unvec(const VectorXd& v, int rows, int cols) {
    MatrixXd M(rows, cols);
    int k = 0;
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            M(i, j) = v(k++);
        }
    }
    return M;
}

MatrixXd solveContinuousLyapunov(const MatrixXd& A, const MatrixXd& C) {
    // Solves A^T P + P A + C = 0.
    const int n = A.rows();
    MatrixXd I = MatrixXd::Identity(n, n);
    MatrixXd L = kron(I, A.transpose()) + kron(A.transpose(), I);
    VectorXd rhs = -vec(C);
    VectorXd sol = L.fullPivLu().solve(rhs);
    MatrixXd P = unvec(sol, n, n);
    return 0.5 * (P + P.transpose());
}

MatrixXd rk4Step(const MatrixXd& Acl, const MatrixXd& x, double h) {
    MatrixXd k1 = Acl * x;
    MatrixXd k2 = Acl * (x + 0.5 * h * k1);
    MatrixXd k3 = Acl * (x + 0.5 * h * k2);
    MatrixXd k4 = Acl * (x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main() {
    MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    MatrixXd B(2, 1);
    B << 0.0,
         1.0;

    MatrixXd Q = MatrixXd::Zero(2, 2);
    Q(0, 0) = 10.0;
    Q(1, 1) = 1.0;

    MatrixXd R(1, 1);
    R << 0.2;

    // Initial stabilizing gain for the double integrator.
    MatrixXd K(1, 2);
    K << 2.0, 2.0;

    MatrixXd Rinv = R.inverse();
    MatrixXd P = MatrixXd::Zero(2, 2);

    for (int iter = 0; iter < 50; ++iter) {
        MatrixXd Acl = A - B * K;
        MatrixXd C = Q + K.transpose() * R * K;
        MatrixXd Pnew = solveContinuousLyapunov(Acl, C);
        MatrixXd Knew = Rinv * B.transpose() * Pnew;

        double err = (Knew - K).norm();
        K = Knew;
        P = Pnew;

        if (err < 1e-11) {
            break;
        }
    }

    MatrixXd Acl = A - B * K;
    MatrixXd x(2, 1);
    x << 1.0,
         0.0;

    double h = 0.002;
    int steps = static_cast<int>(8.0 / h);
    double J = 0.0;

    for (int i = 0; i < steps; ++i) {
        MatrixXd u = -K * x;
        double integrand = (x.transpose() * Q * x)(0, 0) + (u.transpose() * R * u)(0, 0);
        J += integrand * h;
        x = rk4Step(Acl, x, h);
    }

    std::cout << std::setprecision(8);
    std::cout << "P =\n" << P << "\n\n";
    std::cout << "K =\n" << K << "\n\n";
    std::cout << "finite-horizon estimated cost over [0, 8] = " << J << "\n";
    std::cout << "infinite-horizon value x0^T P x0 = " << P(0, 0) << "\n";

    return 0;
}
