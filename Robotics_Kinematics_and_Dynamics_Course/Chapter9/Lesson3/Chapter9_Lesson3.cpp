#include <iostream>
#include <Eigen/Dense>

// Planar 2R translational Jacobian
Eigen::Matrix2d jacobian2R(double q1, double q2, double l1, double l2) {
    double s1  = std::sin(q1);
    double c1  = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Eigen::Matrix2d J;
    J(0,0) = -l1 * s1 - l2 * s12;
    J(0,1) = -l2 * s12;
    J(1,0) =  l1 * c1 + l2 * c12;
    J(1,1) =  l2 * c12;
    return J;
}

Eigen::Vector2d torqueFromForce(
    double q1, double q2,
    double Fx, double Fy,
    double l1, double l2)
{
    Eigen::Matrix2d J = jacobian2R(q1, q2, l1, l2);
    Eigen::Vector2d f(Fx, Fy);
    Eigen::Vector2d tau = J.transpose() * f;
    return tau;
}

int main() {
    double l1 = 1.0, l2 = 1.0;
    double q1 = 0.0, q2 = 0.0;
    double Fx = 0.0, Fy = -10.0;

    Eigen::Vector2d tau = torqueFromForce(q1, q2, Fx, Fy, l1, l2);
    std::cout << "tau = " << tau.transpose() << std::endl;

    // In Orocos KDL, a general 6 x n Jacobian J can be computed for a kinematic chain.
    // Then one directly uses: tau = J.data.transpose() * F;  where F is a 6D wrench.
    return 0;
}
      
