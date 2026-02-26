#include <Eigen/Dense>
#include <cmath>

static const double L1 = 1.0;
static const double L2 = 1.0;

Eigen::Vector2d fk_planar_2R(const Eigen::Vector2d& q) {
    double theta1 = q(0);
    double theta2 = q(1);
    double x = L1 * std::cos(theta1) + L2 * std::cos(theta1 + theta2);
    double y = L1 * std::sin(theta1) + L2 * std::sin(theta1 + theta2);
    return Eigen::Vector2d(x, y);
}

Eigen::Matrix2d jacobian_fk(const Eigen::Vector2d& q) {
    double theta1 = q(0);
    double theta2 = q(1);
    double s1  = std::sin(theta1);
    double c1  = std::cos(theta1);
    double s12 = std::sin(theta1 + theta2);
    double c12 = std::cos(theta1 + theta2);

    Eigen::Matrix2d J;
    J(0,0) = -L1 * s1 - L2 * s12;
    J(0,1) = -L2 * s12;
    J(1,0) =  L1 * c1 + L2 * c12;
    J(1,1) =  L2 * c12;
    return J;
}

Eigen::Vector2d projectOntoTaskManifold(
    const Eigen::Vector2d& q_init,
    const Eigen::Vector2d& p_des,
    int max_iters = 20,
    double tol = 1e-10)
{
    Eigen::Vector2d q = q_init;
    for (int k = 0; k < max_iters; ++k) {
        Eigen::Vector2d h = fk_planar_2R(q) - p_des;
        Eigen::Matrix2d J = jacobian_fk(q);
        // For a full-rank 2x2, pseudoinverse reduces to this expression:
        Eigen::Matrix2d J_pinv = J.transpose() * (J * J.transpose()).inverse();
        q -= J_pinv * h;
        if (h.norm() < tol) {
            break;
        }
    }
    return q;
}
      
