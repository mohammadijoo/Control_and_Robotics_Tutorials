
#include <Eigen/Dense>

// These would typically be provided by RBDL, Pinocchio, or a similar library.
void computeInertia(const Eigen::VectorXd& q, Eigen::MatrixXd& M);
void computeCoriolis(const Eigen::VectorXd& q,
                     const Eigen::VectorXd& qdot,
                     Eigen::MatrixXd& C);
void computeGravity(const Eigen::VectorXd& q, Eigen::VectorXd& g);

Eigen::VectorXd feedbackLinearization(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qd_dot,
    const Eigen::VectorXd& qd_ddot,
    const Eigen::MatrixXd& Kp,
    const Eigen::MatrixXd& Kv)
{
    Eigen::VectorXd e = q - qd;
    Eigen::VectorXd e_dot = qdot - qd_dot;

    Eigen::VectorXd v = qd_ddot - Kv * e_dot - Kp * e;

    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    Eigen::VectorXd g;

    computeInertia(q, M);
    computeCoriolis(q, qdot, C);
    computeGravity(q, g);

    Eigen::VectorXd tau = M * v + C * qdot + g;
    return tau;
}
