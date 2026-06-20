
#include <Eigen/Dense>
// Include your QP solver headers here, e.g. qpOASES or OSQP

// Example: build H and g for a weighted QP from two tasks
void buildWeightedQP(const Eigen::MatrixXd& J1,
                     const Eigen::VectorXd& b1,
                     const Eigen::MatrixXd& J2,
                     const Eigen::VectorXd& b2,
                     double alpha1,
                     double alpha2,
                     Eigen::MatrixXd& H,
                     Eigen::VectorXd& g)
{
    const int m = J1.cols();
    Eigen::MatrixXd W1 = Eigen::MatrixXd::Identity(J1.rows(), J1.rows());
    Eigen::MatrixXd W2 = Eigen::MatrixXd::Identity(J2.rows(), J2.rows());
    double lambda_reg = 1e-6;

    H = alpha1 * J1.transpose() * W1 * J1
      + alpha2 * J2.transpose() * W2 * J2
      + lambda_reg * Eigen::MatrixXd::Identity(m, m);

    g = -(alpha1 * J1.transpose() * W1 * b1
        + alpha2 * J2.transpose() * W2 * b2);
}

// In your control loop:
// 1. Use Pinocchio/RBDL to compute J1, J2 from the current q, qdot.
// 2. Call buildWeightedQP.
// 3. Solve the QP and map the solution vector to accelerations, torques, etc.
