
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// Suppose we have functions (from RBDL or your own code):
//   MatrixXd M_mat(const VectorXd& q);
//   MatrixXd C_mat(const VectorXd& q, const VectorXd& qd);
//   VectorXd g_vec(const VectorXd& q);

VectorXd computedTorque(
    const VectorXd& q,
    const VectorXd& qd,
    const VectorXd& q_d,
    const VectorXd& qd_d,
    const VectorXd& qdd_d,
    const MatrixXd& Kp,
    const MatrixXd& Kd)
{
    VectorXd e  = q  - q_d;
    VectorXd ed = qd - qd_d;
    VectorXd v  = qdd_d - Kd * ed - Kp * e;

    MatrixXd M = M_mat(q);
    MatrixXd C = C_mat(q, qd);
    VectorXd g = g_vec(q);

    VectorXd tau = M * v + C * qd + g;
    return tau;
}
