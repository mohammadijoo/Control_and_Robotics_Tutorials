#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct PendulumParams {
    double m;
    double L;
    double b;
    double g;
};

void linearizePendulum(const PendulumParams& p,
                       double theta_op,
                       double omega_op,
                       double u_op,
                       Matrix2d& A,
                       Vector2d& B)
{
    // For the simple pendulum, the Jacobians are analytically known.
    (void)u_op; // unused: equilibrium torque is zero here

    A.setZero();
    A(0,0) = 0.0;
    A(0,1) = 1.0;
    A(1,0) = -p.g / p.L * std::cos(theta_op);
    A(1,1) = -p.b / (p.m * p.L * p.L);

    B(0) = 0.0;
    B(1) = 1.0 / (p.m * p.L * p.L);
}

int main() {
    PendulumParams p{1.0, 1.0, 0.1, 9.81};

    Matrix2d A;
    Vector2d B;

    double theta_op = 0.0;
    double omega_op = 0.0;
    double u_op     = 0.0;

    linearizePendulum(p, theta_op, omega_op, u_op, A, B);

    std::cout << "A =\n" << A << "\n\n";
    std::cout << "B =\n" << B << "\n";

    return 0;
}
