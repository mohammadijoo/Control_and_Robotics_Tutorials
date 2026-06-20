
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    const int nx = 4;
    const int nu = 2;
    const int ny = 2;
    const int N  = 10;

    MatrixXd A = MatrixXd::Identity(nx, nx);
    A += 0.01 * MatrixXd::Random(nx, nx);
    MatrixXd B = 0.01 * MatrixXd::Random(nx, nu);
    MatrixXd C(ny, nx);
    C << 1,0,0,0,
          0,1,0,0;

    // Build prediction matrices
    MatrixXd Sx = MatrixXd::Zero(N * nx, nx);
    MatrixXd Su = MatrixXd::Zero(N * nx, N * nu);

    MatrixXd A_power = MatrixXd::Identity(nx, nx);
    for (int i = 0; i < N; ++i) {
        A_power = A_power * A; // A^(i+1)
        Sx.block(i*nx, 0, nx, nx) = A_power;

        for (int j = 0; j <= i; ++j) {
            MatrixXd A_pow = MatrixXd::Identity(nx, nx);
            for (int p = 0; p < (i - j); ++p) {
                A_pow = A_pow * A;
            }
            Su.block(i*nx, j*nu, nx, nu) = A_pow * B;
        }
    }

    // Block-diagonal C and weights
    MatrixXd C_blk = MatrixXd::Zero(N * ny, N * nx);
    for (int i = 0; i < N; ++i) {
        C_blk.block(i*ny, i*nx, ny, nx) = C;
    }

    MatrixXd Qy = MatrixXd::Identity(ny, ny);
    MatrixXd R  = 0.01 * MatrixXd::Identity(nu, nu);

    MatrixXd Q_blk = MatrixXd::Zero(N * ny, N * ny);
    MatrixXd R_blk = MatrixXd::Zero(N * nu, N * nu);
    for (int i = 0; i < N; ++i) {
        Q_blk.block(i*ny, i*ny, ny, ny) = Qy;
        R_blk.block(i*nu, i*nu, nu, nu) = R;
    }

    MatrixXd T = C_blk * Su;

    // One MPC step
    VectorXd xk = VectorXd::Zero(nx);
    VectorXd y_ref = VectorXd::Zero(N * ny); // zero reference

    VectorXd b = C_blk * (Sx * xk) - y_ref;

    MatrixXd H = 2.0 * (T.transpose() * Q_blk * T + R_blk);
    VectorXd f = 2.0 * (T.transpose() * Q_blk * b);

    // Solve H U = -f (unconstrained case)
    VectorXd U_star = H.ldlt().solve(-f);
    VectorXd u0_star = U_star.head(nu);

    std::cout << "u0* = " << u0_star.transpose() << std::endl;
    return 0;
}
