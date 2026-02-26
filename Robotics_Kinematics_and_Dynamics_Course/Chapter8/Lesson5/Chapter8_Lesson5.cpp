#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix2d;
using Eigen::Vector2d;

Matrix2d jacobian2R(const Vector2d& q, double l1 = 1.0, double l2 = 1.0) {
    double q1 = q(0), q2 = q(1);
    double s1  = std::sin(q1);
    double c1  = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Matrix2d J;
    J(0, 0) = -l1 * s1 - l2 * s12;
    J(0, 1) = -l2 * s12;
    J(1, 0) =  l1 * c1 + l2 * c12;
    J(1, 1) =  l2 * c12;
    return J;
}

double manipulability(const Matrix2d& J) {
    Matrix2d JJt = J * J.transpose();
    return std::sqrt(JJt.determinant());
}

double jointLimitPenalty(const Vector2d& q,
                         const Vector2d& qmin,
                         const Vector2d& qmax) {
    double phi = 0.0;
    for (int i = 0; i < 2; ++i) {
        double qi = q(i);
        double ql = qmin(i);
        double qu = qmax(i);
        phi += 1.0 / std::pow(qi - ql, 2.0)
             + 1.0 / std::pow(qu - qi, 2.0);
    }
    return phi;
}

double workspacePenalty(const Vector2d& q,
                        double l1 = 1.0, double l2 = 1.0,
                        double eps_ws = 1e-3) {
    double q1 = q(0), q2 = q(1);
    double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
    double r = std::sqrt(x * x + y * y);

    double rmin = std::fabs(l1 - l2);
    double rmax = l1 + l2;
    double d_ws = std::min(r - rmin, rmax - r);
    return 1.0 / (d_ws * d_ws + eps_ws);
}

double avoidanceCost(const Vector2d& q,
                     double l1 = 1.0, double l2 = 1.0,
                     double eps = 1e-3) {
    Matrix2d J = jacobian2R(q, l1, l2);
    double w = manipulability(J);
    double phi_sing = 1.0 / (w * w + eps);

    Vector2d qmin(-M_PI, -M_PI);
    Vector2d qmax( M_PI,  M_PI);
    double phi_joint = jointLimitPenalty(q, qmin, qmax);
    double phi_ws = workspacePenalty(q, l1, l2);
    return phi_sing + phi_joint + phi_ws;
}

int main() {
    Vector2d q(0.0, 0.5);
    for (int k = 0; k < 10; ++k) {
        double c = avoidanceCost(q);
        std::cout << "q = [" << q.transpose()
                  << "], cost = " << c << std::endl;
        // Offline finite-difference gradient step could be added here
    }
    return 0;
}
      
