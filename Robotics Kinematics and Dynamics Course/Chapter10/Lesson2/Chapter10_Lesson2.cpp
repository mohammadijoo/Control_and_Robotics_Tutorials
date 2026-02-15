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

Vector2d generalizedTorquesFromForce(double q1, double q2,
                                     double l1, double l2,
                                     const Vector2d &f) {
    Matrix2d J = jacobian2R(q1, q2, l1, l2);
    return J.transpose() * f;
}

int main() {
    double l1 = 1.0, l2 = 0.8;
    double q1 = 0.5, q2 = -0.3;
    Vector2d f;
    f << 10.0, 5.0;

    Vector2d tau = generalizedTorquesFromForce(q1, q2, l1, l2, f);
    std::cout << "Joint torques: " << tau.transpose() << std::endl;

    return 0;
}
      
