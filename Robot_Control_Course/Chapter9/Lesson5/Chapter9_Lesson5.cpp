
#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Physical parameters
double m = 1.0;
double ell = 1.0;
double Iinertia = m * ell * ell;
double b = 0.1;
double g = 9.81;
double dt = 0.01;

// Continuous-time dynamics xdot = f(x,u) for single-link
Vector2d dynamics(const Vector2d& x, double u) {
    double q = x(0);
    double dq = x(1);
    double ddq = (u - b * dq - m * g * ell * std::sin(q)) / Iinertia;
    Vector2d xdot;
    xdot << dq, ddq;
    return xdot;
}

Vector2d step(const Vector2d& x, double u) {
    return x + dt * dynamics(x, u);
}

int main() {
    // Equilibrium at q* = 0, dq* = 0
    double q_star = 0.0;
    double dq_star = 0.0;
    double u_star = m * g * ell * std::sin(q_star);
    Vector2d x_star(q_star, dq_star);

    // Linearization matrices
    Matrix2d A_c;
    A_c << 0.0, 1.0,
             -(m * g * ell / Iinertia) * std::cos(q_star), -b / Iinertia;
    Eigen::Vector2d B_c_vec(0.0, 1.0 / Iinertia);
    MatrixXd B_c(2, 1);
    B_c.col(0) = B_c_vec;

    Matrix2d A_d = Matrix2d::Identity() + dt * A_c;
    MatrixXd B_d = dt * B_c;

    // Cost matrices
    Matrix2d Q;
    Q << 10.0, 0.0,
           0.0, 1.0;
    Matrix2d Qf;
    Qf << 50.0, 0.0,
            0.0, 5.0;
    Eigen::MatrixXd R(1, 1);
    R(0, 0) = 0.1;

    int N = 500;
    std::vector<Matrix2d> P(N + 1);
    std::vector<Eigen::RowVector2d> K(N);

    P[N] = Qf;
    for (int k = N - 1; k >= 0; --k) {
        Eigen::MatrixXd S = R + B_d.transpose() * P[k + 1] * B_d;
        Eigen::RowVector2d Kk =
            -S.ldlt().solve(B_d.transpose() * P[k + 1] * A_d);
        K[k] = Kk;
        Matrix2d Acl = A_d + B_d * Kk;
        P[k] = Q + Acl.transpose() * P[k + 1] * Acl;
    }

    // Simulate LQR stabilization
    Vector2d x;
    x << 0.5, 0.0;  // perturbed initial angle
    for (int k = 0; k < 500; ++k) {
        Vector2d dx = x - x_star;
        double u = u_star + (K[std::min(k, N - 1)] * dx)(0);
        x = step(x, u);
        std::cout << k * dt << " " << x(0) << " " << x(1) << " " << u << "\n";
    }
    return 0;
}
