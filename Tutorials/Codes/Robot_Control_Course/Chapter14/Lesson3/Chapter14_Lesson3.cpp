
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd dynConsistentPinv(const MatrixXd& M, const MatrixXd& Jc)
{
    // Jc_dyn_pinv = M^{-1} Jc^T (Jc M^{-1} Jc^T)^{-1}
    MatrixXd Minv = M.inverse();
    MatrixXd temp = Jc * Minv * Jc.transpose();
    MatrixXd Lambda_c = temp.inverse();
    MatrixXd Jc_dyn_pinv = Minv * Jc.transpose() * Lambda_c;
    return Jc_dyn_pinv;
}

void contactProjector(const MatrixXd& M,
                      const MatrixXd& Jc,
                      MatrixXd& Nc,
                      MatrixXd& Jc_dyn_pinv)
{
    int n = static_cast<int>(M.rows());
    Jc_dyn_pinv = dynConsistentPinv(M, Jc);
    Nc = MatrixXd::Identity(n, n) - Jc_dyn_pinv * Jc;
}

VectorXd contactConsistentAcc(const MatrixXd& M,
                              const MatrixXd& Jc,
                              const VectorXd& Jc_dot_qdot,
                              const VectorXd& qdd_des)
{
    MatrixXd Nc, Jc_dyn_pinv;
    contactProjector(M, Jc, Nc, Jc_dyn_pinv);
    // qdd_star = Nc qdd_des - Jc_dyn_pinv Jc_dot_qdot
    return Nc * qdd_des - Jc_dyn_pinv * Jc_dot_qdot;
}
