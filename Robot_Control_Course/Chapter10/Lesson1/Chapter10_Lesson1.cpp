
#include <iostream>
#include <Eigen/Dense>

// For multi-DOF robots, obtain A,B from RBDL or Pinocchio.

using namespace Eigen;

int main() {
    const double Ts = 0.02;
    const double I  = 1.0;

    Matrix2d A;
    A << 1.0, Ts,
           0.0, 1.0;
    Vector2d B;
    B << 0.5 * Ts * Ts / I,
           Ts / I;

    Matrix2d Q;
    Q << 10.0, 0.0,
           0.0,  1.0;
    MatrixXd R(1,1);
    R(0,0) = 0.1;
    Matrix2d P = Q; // simple choice

    const int N = 20;
    const int n = 2;
    const int m = 1;

    MatrixXd A_bar = MatrixXd::Zero(N * n, n);
    MatrixXd B_bar = MatrixXd::Zero(N * n, N * m);

    for (int i = 0; i < N; ++i) {
        Matrix2d A_power = Matrix2d::Identity();
        for (int k = 0; k < i + 1; ++k) {
            A_power = A_power * A;
        }
        A_bar.block(i*n, 0, n, n) = A_power;

        for (int j = 0; j <= i; ++j) {
            Matrix2d A_ij = Matrix2d::Identity();
            for (int k = 0; k < i - j; ++k) {
                A_ij = A_ij * A;
            }
            B_bar.block(i*n, j*m, n, m) = A_ij * B;
        }
    }

    // Build Q_bar and R_bar
    MatrixXd Q_bar = MatrixXd::Zero(N * n, N * n);
    for (int i = 0; i < N - 1; ++i) {
        Q_bar.block(i*n, i*n, n, n) = Q;
    }
    Q_bar.block((N-1)*n, (N-1)*n, n, n) = P;

    MatrixXd R_bar = MatrixXd::Zero(N * m, N * m);
    for (int i = 0; i < N; ++i) {
        R_bar.block(i*m, i*m, m, m) = R;
    }

    MatrixXd H = B_bar.transpose() * Q_bar * B_bar + R_bar;
    MatrixXd F = B_bar.transpose() * Q_bar * A_bar;

    // Precompute LDLT factorization of H
    Eigen::LDLT<MatrixXd> ldlt(H);

    auto mpc_control = [&](const Vector2d& x) {
        VectorXd rhs = F * x;
        VectorXd U_star = - ldlt.solve(rhs);
        double u_k = U_star(0);
        return u_k;
    };

    double T_final = 1.0;
    int N_steps = static_cast<int>(T_final / Ts);

    Vector2d x;
    x << 0.5, 0.0;

    for (int k = 0; k < N_steps; ++k) {
        double u = mpc_control(x);
        x = A * x + B * u;
        std::cout << "k=" << k
                  << " q=" << x(0)
                  << " qdot=" << x(1)
                  << " u=" << u << std::endl;
    }

    return 0;
}
