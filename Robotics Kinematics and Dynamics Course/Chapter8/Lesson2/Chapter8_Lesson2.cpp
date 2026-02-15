#include <Eigen/Dense>
#include <iostream>
#include <cmath>

double yoshikawaManipulability(const Eigen::MatrixXd& J, double tol = 1e-6)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();

    double w = 1.0;
    int rank = 0;
    for (int i = 0; i < s.size(); ++i)
    {
        if (s(i) > tol)
        {
            w *= s(i);
            ++rank;
        }
    }
    if (rank == 0)
    {
        return 0.0;
    }
    return w;
}

Eigen::Matrix2d jacobian2R(double theta1, double theta2, double l1, double l2)
{
    double s1 = std::sin(theta1);
    double c1 = std::cos(theta1);
    double s12 = std::sin(theta1 + theta2);
    double c12 = std::cos(theta1 + theta2);

    Eigen::Matrix2d J;
    J(0, 0) = -l1 * s1 - l2 * s12;
    J(0, 1) = -l2 * s12;
    J(1, 0) =  l1 * c1 + l2 * c12;
    J(1, 1) =  l2 * c12;
    return J;
}

int main()
{
    double l1 = 1.0;
    double l2 = 0.7;
    double theta1 = 0.5;
    double theta2 = 1.0;

    Eigen::Matrix2d J = jacobian2R(theta1, theta2, l1, l2);
    double w = yoshikawaManipulability(J);

    std::cout << "J =\n" << J << std::endl;
    std::cout << "Yoshikawa manipulability w = " << w << std::endl;

    // In a ROS + KDL workflow, you would obtain J from a KDL::ChainJntToJacSolver
    // and then convert to Eigen::MatrixXd before calling yoshikawaManipulability.
    return 0;
}
      
