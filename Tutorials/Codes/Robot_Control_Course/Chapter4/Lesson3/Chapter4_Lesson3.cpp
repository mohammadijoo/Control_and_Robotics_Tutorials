
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd pseudoinverse(const MatrixXd& J, double damping = 0.0) {
    int m = J.rows();
    int n = J.cols();
    if (damping <= 0.0) {
        MatrixXd JJt = J * J.transpose();
        return J.transpose() * JJt.inverse();
    } else {
        MatrixXd JJt = J * J.transpose();
        MatrixXd I = MatrixXd::Identity(m, m);
        return J.transpose() * (JJt + damping * damping * I).inverse();
    }
}

MatrixXd nullProjector(const MatrixXd& J, double damping = 0.0) {
    MatrixXd Jsharp = pseudoinverse(J, damping);
    int n = J.cols();
    MatrixXd I = MatrixXd::Identity(n, n);
    return I - Jsharp * J;
}

VectorXd velocityLevelControl(
    const VectorXd& q,
    const VectorXd& x,
    const VectorXd& xdot,
    const VectorXd& x_d,
    const VectorXd& xdot_d_ff,
    const MatrixXd& Kp,
    const MatrixXd& Kd,
    const VectorXd& grad_h,
    double k_h,
    const MatrixXd& J,
    double damping = 0.0)
{
    // Task-space velocity command
    VectorXd x_error = x_d - x;
    VectorXd xdot_error = xdot_d_ff - xdot;
    VectorXd xdot_d = xdot_d_ff + Kp * x_error + Kd * xdot_error;

    MatrixXd Jsharp = pseudoinverse(J, damping);
    MatrixXd N = nullProjector(J, damping);

    VectorXd qdot_task = Jsharp * xdot_d;
    VectorXd qdot_null = -k_h * grad_h;

    VectorXd qdot = qdot_task + N * qdot_null;
    return qdot;
}
