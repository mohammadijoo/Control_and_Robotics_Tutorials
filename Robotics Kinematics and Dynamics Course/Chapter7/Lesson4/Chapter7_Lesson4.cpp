#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

Vector2d planar3r_fk(const Vector3d& q, const Vector3d& L) {
    double q1 = q(0), q2 = q(1), q3 = q(2);
    double l1 = L(0), l2 = L(1), l3 = L(2);

    double c1 = std::cos(q1), s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2), s12 = std::sin(q1 + q2);
    double c123 = std::cos(q1 + q2 + q3), s123 = std::sin(q1 + q2 + q3);

    double x = l1 * c1 + l2 * c12 + l3 * c123;
    double y = l1 * s1 + l2 * s12 + l3 * s123;
    return Vector2d(x, y);
}

Eigen::Matrix<double, 2, 3> planar3r_jacobian(const Vector3d& q,
                                                const Vector3d& L)
{
    double q1 = q(0), q2 = q(1), q3 = q(2);
    double l1 = L(0), l2 = L(1), l3 = L(2);

    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2), c12 = std::cos(q1 + q2);
    double s123 = std::sin(q1 + q2 + q3), c123 = std::cos(q1 + q2 + q3);

    double dx_dq1 = -l1 * s1 - l2 * s12 - l3 * s123;
    double dx_dq2 = -l2 * s12 - l3 * s123;
    double dx_dq3 = -l3 * s123;

    double dy_dq1 =  l1 * c1 + l2 * c12 + l3 * c123;
    double dy_dq2 =  l2 * c12 + l3 * c123;
    double dy_dq3 =  l3 * c123;

    Eigen::Matrix<double, 2, 3> J;
    J << dx_dq1, dx_dq2, dx_dq3,
          dy_dq1, dy_dq2, dy_dq3;
    return J;
}

Vector3d minimum_norm_qdot(const Vector3d& q,
                           const Vector3d& L,
                           const Vector2d& xdot_des)
{
    auto J = planar3r_jacobian(q, L);      // 2x3
    Matrix2d JJt = J * J.transpose();      // 2x2
    Eigen::Matrix<double, 3, 2> J_pinv = J.transpose() * JJt.inverse();
    return J_pinv * xdot_des;
}

int main() {
    Vector3d L(0.5, 0.4, 0.3);
    Vector3d q(0.2, 0.5, -0.4);
    Vector2d xdot_des(0.05, -0.02);

    Vector3d qdot = minimum_norm_qdot(q, L, xdot_des);
    std::cout << "qdot = " << qdot.transpose() << std::endl;
    return 0;
}
      
