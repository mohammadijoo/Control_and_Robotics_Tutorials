#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

Matrix2d jacobian2R(double q1, double q2, double l1, double l2) {
    double s1  = std::sin(q1);
    double c1  = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);
    Matrix2d J;
    J(0,0) = -l1 * s1 - l2 * s12;
    J(0,1) = -l2 * s12;
    J(1,0) =  l1 * c1 + l2 * c12;
    J(1,1) =  l2 * c12;
    return J;
}

Matrix2d jdot2R(double q1, double q2,
                double dq1, double dq2,
                double l1, double l2) {
    double c1  = std::cos(q1);
    double s1  = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);
    Matrix2d Jdot;
    Jdot(0,0) = -dq1 * l1 * c1 - (dq1 + dq2) * l2 * c12;
    Jdot(0,1) = -(dq1 + dq2) * l2 * c12;
    Jdot(1,0) = -dq1 * l1 * s1 - (dq1 + dq2) * l2 * s12;
    Jdot(1,1) = -(dq1 + dq2) * l2 * s12;
    return Jdot;
}

int main() {
    double l1 = 1.0, l2 = 0.7;
    double q1 = 30.0 * M_PI / 180.0;
    double q2 = 20.0 * M_PI / 180.0;
    double dq1 = 0.5, dq2 = -0.3;
    double ddq1 = 0.2, ddq2 = 0.1;

    Matrix2d J = jacobian2R(q1, q2, l1, l2);
    Matrix2d Jdot = jdot2R(q1, q2, dq1, dq2, l1, l2);

    Vector2d dq(dq1, dq2);
    Vector2d ddq(ddq1, ddq2);

    Vector2d xdot = J * dq;
    Vector2d xddot = J * ddq + Jdot * dq;

    std::cout << "xdot = " << xdot.transpose() << std::endl;
    std::cout << "xddot = " << xddot.transpose() << std::endl;
    return 0;
}
      
