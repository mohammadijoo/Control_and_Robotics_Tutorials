
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct OpSpaceGains {
    MatrixXd Kp;
    MatrixXd Kd;
};

VectorXd opSpaceControlStep(
    const VectorXd& q,
    const VectorXd& qd,
    const VectorXd& x,
    const VectorXd& xd,
    const VectorXd& x_d,
    const VectorXd& xd_d,
    const VectorXd& xdd_d,
    const OpSpaceGains& gains,
    const MatrixXd& M,
    const VectorXd& h,
    const MatrixXd& J,
    const MatrixXd& Jdot)
{
    const int m = x.size();

    // Errors
    VectorXd e  = x  - x_d;
    VectorXd ed = xd - xd_d;

    // Desired task acceleration
    VectorXd xdd_ref = xdd_d
                       - gains.Kd * ed
                       - gains.Kp * e;

    // Lambda = (J M^-1 J^T)^-1
    MatrixXd Minv = M.inverse();
    MatrixXd JMJT = J * Minv * J.transpose();
    MatrixXd Lambda = JMJT.inverse();

    // Task-space mu and p (gravity could be split if available)
    VectorXd mu = Lambda * (J * Minv * h) - Lambda * (Jdot * qd);
    VectorXd p  = VectorXd::Zero(m); // if gravity already in h

    VectorXd F = Lambda * xdd_ref + mu + p;

    // tau = J^T F
    VectorXd tau = J.transpose() * F;
    return tau;
}
