#include <iostream>
#include <Eigen/Dense>

Eigen::Matrix3d planar2RJacobian(double l1, double l2,
                                 double q1, double q2)
{
    Eigen::Matrix3d J;
    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    // row 0: omega_z contributions
    J(0, 0) = 1.0;
    J(0, 1) = 1.0;

    // row 1: v_x
    J(1, 0) = -l1 * s1 - l2 * s12;
    J(1, 1) = -l2 * s12;

    // row 2: v_y
    J(2, 0) =  l1 * c1 + l2 * c12;
    J(2, 1) =  l2 * c12;

    return J;
}

int main()
{
    double l1 = 1.0, l2 = 0.8;
    double q1 = 0.5, q2 = -0.3;
    Eigen::Matrix3d J = planar2RJacobian(l1, l2, q1, q2);
    std::cout << "Planar Jacobian:\n" << J << std::endl;
    return 0;
}
      
