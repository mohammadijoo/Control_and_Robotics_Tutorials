
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct ConstrainedTorqueResult {
    VectorXd tau;
    VectorXd lambda;
};

ConstrainedTorqueResult computeProjectedTorque(
    const MatrixXd& M,
    const VectorXd& h,
    const MatrixXd& Jc,
    const VectorXd& bc,
    const VectorXd& tau0,
    double eps = 1e-9)
{
    const int n = M.rows();
    const int m = Jc.rows();

    // Solve M x = (tau0 - h)
    VectorXd Minv_tau0_minus_h = M.ldlt().solve(tau0 - h);

    // Compute Jc * M^{-1} * Jc^T
    MatrixXd Minv_JcT = M.ldlt().solve(Jc.transpose());      // (n,m)
    MatrixXd JMJT = Jc * Minv_JcT;                           // (m,m)

    // Regularize and invert
    MatrixXd JMJT_reg = JMJT + eps * MatrixXd::Identity(m, m);
    MatrixXd Lambda_c = JMJT_reg.inverse();

    // Lagrange multipliers
    VectorXd lambda = -Lambda_c * (Jc * Minv_tau0_minus_h + bc);

    // Final torque
    VectorXd tau = tau0 + Jc.transpose() * lambda;

    return {tau, lambda};
}

// Example usage inside control loop (using Pinocchio or RBDL to get M, h, Jc, bc)
void controllerStep(const VectorXd& q,
                    const VectorXd& qd,
                    const VectorXd& q_ref,
                    const VectorXd& qd_ref,
                    const VectorXd& qdd_ref,
                    VectorXd& tau_out)
{
    MatrixXd M;
    VectorXd h;
    // e.g. pinocchio::crba(model, data, q); pinocchio::nonLinearEffects(...);
    getMassMatrixAndBias(q, qd, M, h);

    MatrixXd Jc;
    VectorXd bc;
    getConstraintJacobianAndBias(q, qd, Jc, bc);

    // Unconstrained PD + feedforward
    MatrixXd Kp = MatrixXd::Identity(q.size(), q.size()) * 100.0;
    MatrixXd Kd = MatrixXd::Identity(q.size(), q.size()) * 20.0;

    VectorXd e  = q_ref  - q;
    VectorXd ed = qd_ref - qd;
    VectorXd v  = qdd_ref + Kd * ed + Kp * e;
    VectorXd tau0 = M * v + h;

    auto res = computeProjectedTorque(M, h, Jc, bc, tau0);
    tau_out = res.tau;
}
