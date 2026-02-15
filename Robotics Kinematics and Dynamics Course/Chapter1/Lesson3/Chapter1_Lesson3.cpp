#include <Eigen/Dense>
#include <cmath>

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

// Forward kinematics
Vec2 fk2R(double q1, double q2, double l1, double l2) {
    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    double x = l1 * c1 + l2 * c12;
    double y = l1 * s1 + l2 * s12;
    return Vec2(x, y);
}

// Analytic Jacobian
Mat2 jac2R(double q1, double q2, double l1, double l2) {
    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    Mat2 J;
    J(0,0) = -l1 * s1 - l2 * s12;
    J(0,1) = -l2 * s12;
    J(1,0) =  l1 * c1 + l2 * c12;
    J(1,1) =  l2 * c12;
    return J;
}

// Gradient of phi(q) = 0.5 * ||f(q) - x_d||^2
Vec2 gradPhi2R(const Vec2& q, const Vec2& x_d,
               double l1, double l2) {
    Vec2 x = fk2R(q(0), q(1), l1, l2);
    Vec2 e = x - x_d;
    Mat2 J = jac2R(q(0), q(1), l1, l2);
    // grad_phi = J(q)^T * e
    return J.transpose() * e;
}
      
