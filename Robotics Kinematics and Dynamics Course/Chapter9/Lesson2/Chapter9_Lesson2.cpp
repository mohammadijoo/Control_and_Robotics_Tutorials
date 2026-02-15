#include <iostream>
#include <Eigen/Dense>

Eigen::Matrix2d jacobian2R(double q1, double q2, double l1, double l2) {
    double s1 = std::sin(q1);
    double c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Eigen::Matrix2d J;
    J(0,0) = -l1*s1 - l2*s12;
    J(0,1) = -l2*s12;
    J(1,0) =  l1*c1 + l2*c12;
    J(1,1) =  l2*c12;
    return J;
}

Eigen::Vector2d jointTorquesFromForce(double q1, double q2,
                                      double l1, double l2,
                                      double fx, double fy) {
    Eigen::Matrix2d J = jacobian2R(q1, q2, l1, l2);
    Eigen::Vector2d f(fx, fy);
    return J.transpose() * f;
}

int main() {
    double q1 = M_PI/4.0;
    double q2 = M_PI/6.0;
    double l1 = 1.0, l2 = 0.8;
    double fx = 10.0, fy = 0.0;

    Eigen::Vector2d tau = jointTorquesFromForce(q1, q2, l1, l2, fx, fy);
    std::cout << "tau = " << tau.transpose() << std::endl;
    return 0;
}
      
