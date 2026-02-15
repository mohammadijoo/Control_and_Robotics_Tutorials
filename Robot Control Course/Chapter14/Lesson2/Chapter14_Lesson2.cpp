
#include <Eigen/Dense>
// Include your favorite QP solver headers, e.g. qpOASES, OSQP, etc.

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main()
{
    const int nq = 7;
    const int mx = 6;

    MatrixXd Jx(mx, nq);  // end-effector Jacobian
    MatrixXd Jp(nq, nq);  // posture task (identity)
    Jx.setRandom();
    Jp.setIdentity();

    VectorXd x_err(mx);
    x_err.setRandom();
    VectorXd xdot_ref(mx);
    xdot_ref.setZero();

    MatrixXd Kx = 5.0 * MatrixXd::Identity(mx, mx);
    MatrixXd Wx = MatrixXd::Identity(mx, mx);
    MatrixXd Wp = 0.1 * MatrixXd::Identity(nq, nq);

    VectorXd qdot_nom = VectorXd::Zero(nq);
    VectorXd qdot_ref = VectorXd::Zero(nq);

    VectorXd v_x = xdot_ref - Kx * x_err;

    // Build H and g for the QP: 0.5 qdot^T H qdot + g^T qdot
    MatrixXd H = MatrixXd::Zero(nq, nq);
    VectorXd g = VectorXd::Zero(nq);

    double w1 = 1000.0;
    double w2 = 1.0;
    double lambda_reg = 1e-3;

    // Level 1 contribution
    H += w1 * Jx.transpose() * Wx.transpose() * Wx * Jx;
    g += -w1 * Jx.transpose() * Wx.transpose() * Wx * v_x;

    // Level 2 (posture) contribution
    H += w2 * Jp.transpose() * Wp.transpose() * Wp * Jp;
    g += -w2 * Jp.transpose() * Wp.transpose() * Wp * qdot_ref;

    // Regularization
    H += lambda_reg * MatrixXd::Identity(nq, nq);
    g += -lambda_reg * qdot_nom;

    // Velocity bounds
    VectorXd qdot_min = -0.5 * VectorXd::Ones(nq);
    VectorXd qdot_max =  0.5 * VectorXd::Ones(nq);

    // Convert to the format expected by your QP solver.
    // For example, qpOASES expects H, g, A, lb, ub, etc.
    // Here we only show the conceptual structure.

    // Example: qdot_min <= qdot <= qdot_max
    // This can be encoded as:
    //  lb = qdot_min
    //  ub = qdot_max

    VectorXd qdot_star(nq);
    // Call your QP solver here, e.g.:
    // solve_qp(H, g, A, lb, ub, qdot_star);

    return 0;
}
