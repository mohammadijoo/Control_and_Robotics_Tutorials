#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::JacobiSVD;
using Eigen::ComputeFullU;
using Eigen::ComputeFullV;

Matrix2d jacobian2R(double q1, double q2, double l1, double l2) {
    double s1 = std::sin(q1);
    double c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2);
    double c12 = std::cos(q1 + q2);

    Matrix2d J;
    J(0, 0) = -l1 * s1 - l2 * s12;
    J(0, 1) = -l2 * s12;
    J(1, 0) =  l1 * c1 + l2 * c12;
    J(1, 1) =  l2 * c12;
    return J;
}

double cond2(const Matrix2d &J) {
    JacobiSVD<Matrix2d> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &S = svd.singularValues();
    double sigma_max = S(0);
    double sigma_min = S(1);
    return sigma_max / sigma_min;
}

int main() {
    double l1 = 1.0, l2 = 1.0;
    double q1 = 0.0, q2 = 0.9;

    Matrix2d J = jacobian2R(q1, q2, l1, l2);
    double kappa = cond2(J);

    std::cout << "Jacobian J:\n" << J << "\n\n";
    std::cout << "Condition number kappa_2(J) = " << kappa << std::endl;
    return 0;
}
      
