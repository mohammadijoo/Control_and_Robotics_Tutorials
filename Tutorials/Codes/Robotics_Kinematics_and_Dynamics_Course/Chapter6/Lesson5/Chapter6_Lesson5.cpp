#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd dampedPseudoinverse(const MatrixXd& J, double lambda)
{
    const int m = J.rows();
    MatrixXd JJt = J * J.transpose();
    MatrixXd I = MatrixXd::Identity(m, m);
    MatrixXd inv = (JJt + (lambda * lambda) * I).inverse();
    return J.transpose() * inv;  // J^T (J J^T + lambda^2 I)^(-1)
}

MatrixXd planar3Jacobian(const VectorXd& q,
                         const VectorXd& link_lengths)
{
    double q1 = q(0), q2 = q(1), q3 = q(2);
    double l1 = link_lengths(0), l2 = link_lengths(1), l3 = link_lengths(2);

    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2), c12 = std::cos(q1 + q2);
    double s123 = std::sin(q1 + q2 + q3), c123 = std::cos(q1 + q2 + q3);

    MatrixXd J(2, 3);
    J(0, 0) = -l1 * s1 - l2 * s12 - l3 * s123;
    J(1, 0) =  l1 * c1 + l2 * c12 + l3 * c123;
    J(0, 1) = -l2 * s12 - l3 * s123;
    J(1, 1) =  l2 * c12 + l3 * c123;
    J(0, 2) = -l3 * s123;
    J(1, 2) =  l3 * c123;
    return J;
}

VectorXd jointLimitGradient(const VectorXd& q,
                            const VectorXd& q_min,
                            const VectorXd& q_max)
{
    VectorXd q_mid = 0.5 * (q_min + q_max);
    VectorXd span = q_max - q_min;
    VectorXd normed = (q - q_mid).cwiseQuotient(span);
    // Unit weights for simplicity
    return normed.cwiseQuotient(span);
}

VectorXd redundancyResolutionStep(const VectorXd& q,
                                  const VectorXd& xdot_des,
                                  const VectorXd& link_lengths,
                                  const VectorXd& q_min,
                                  const VectorXd& q_max,
                                  double alpha,
                                  double lambda)
{
    MatrixXd J = planar3Jacobian(q, link_lengths);
    MatrixXd J_pinv = dampedPseudoinverse(J, lambda);

    VectorXd qdot_0 = J_pinv * xdot_des;

    MatrixXd I = MatrixXd::Identity(J.cols(), J.cols());
    MatrixXd N = I - J_pinv * J;

    VectorXd gradH = jointLimitGradient(q, q_min, q_max);
    VectorXd qdot_H = -alpha * (N * gradH);

    VectorXd qdot = qdot_0 + qdot_H;
    return qdot;
}

int main()
{
    VectorXd link_lengths(3);
    link_lengths << 0.4, 0.3, 0.2;

    VectorXd q(3);
    q << 0.0, 0.3, -0.2;

    VectorXd q_min(3), q_max(3);
    q_min << -M_PI, -M_PI, -M_PI;
    q_max <<  M_PI,  M_PI,  M_PI;

    VectorXd xdot_des(2);
    xdot_des << 0.05, 0.0;

    double alpha = 0.2;
    double lambda = 1e-3;

    VectorXd qdot = redundancyResolutionStep(q, xdot_des,
                                             link_lengths,
                                             q_min, q_max,
                                             alpha, lambda);
    VectorXd q_next = q + 0.01 * qdot;

    std::cout << "qdot = " << qdot.transpose() << std::endl;
    std::cout << "q_next = " << q_next.transpose() << std::endl;
    return 0;
}
      
