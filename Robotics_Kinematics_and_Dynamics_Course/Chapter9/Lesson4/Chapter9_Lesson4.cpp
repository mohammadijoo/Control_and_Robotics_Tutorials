#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

Matrix2d planar2RJacobian(double q1, double q2, double l1, double l2) {
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

double directionalCapacityBox(const Matrix2d& J,
                              const Vector2d& f_dir,
                              const Vector2d& tau_max,
                              double eps = 1e-9) {
    double norm = f_dir.norm();
    if (norm < eps) {
        throw std::runtime_error("Direction vector must be nonzero.");
    }
    Vector2d f_hat = f_dir / norm;
    Vector2d v = J.transpose() * f_hat;

    double lambda_star = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 2; ++i) {
        double v_i = v(i);
        if (std::abs(v_i) < eps) {
            continue;  // does not constrain this direction
        }
        double lambda_i = tau_max(i) / std::abs(v_i);
        if (lambda_i < lambda_star) {
            lambda_star = lambda_i;
        }
    }
    return lambda_star;
}

int main() {
    double l1 = 1.0, l2 = 1.0;
    double q1 = 0.0, q2 = 0.0;

    Matrix2d J = planar2RJacobian(q1, q2, l1, l2);
    Vector2d tau_max(10.0, 10.0);
    Vector2d f_dir(0.0, -1.0);

    double lambda_star = directionalCapacityBox(J, f_dir, tau_max);
    std::cout << "Directional capacity lambda* = " << lambda_star << std::endl;
    return 0;
}
      
