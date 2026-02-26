#include <iostream>
#include <Eigen/Dense>
// #include <rbdl/rbdl.h>  // if using RBDL

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct ContactSystem {
    MatrixXd M;  // (n x n)
    VectorXd h;  // (n)
    MatrixXd Jc; // (m x n)
};

ContactSystem buildContactSystem(const VectorXd& q,
                                 const VectorXd& qd) {
    // In a real implementation, call RBDL or Pinocchio here:
    //   RigidBodyDynamics::Model model;
    //   RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, M, update_kinematics);
    //   RigidBodyDynamics::InverseDynamics(model, q, qd, VectorXd::Zero(n), h);
    //   RigidBodyDynamics::CalcPointJacobian6D(model, q, body_id, contact_point, Jc);
    // For illustration we use small constant matrices.
    std::size_t n = 2;
    std::size_t m = 1;
    ContactSystem sys;
    sys.M = MatrixXd::Zero(n, n);
    sys.M(0,0) = 2.0;
    sys.M(1,1) = 1.0;
    sys.M(0,1) = 0.1;
    sys.M(1,0) = 0.1;

    sys.h = VectorXd::Zero(n);
    sys.Jc = MatrixXd::Zero(m, n);
    sys.Jc(0,0) = 1.0;
    sys.Jc(0,1) = 1.0;
    return sys;
}

void solveContactDynamics(const ContactSystem& sys,
                          const VectorXd& tau,
                          const VectorXd& f_ext,
                          const VectorXd& qd,
                          VectorXd& qddot_out,
                          VectorXd& lambda_out) {
    std::size_t n = sys.M.rows();
    std::size_t m = sys.Jc.rows();

    MatrixXd KKT(n + m, n + m);
    KKT.setZero();
    KKT.block(0, 0, n, n) = sys.M;
    KKT.block(0, n, n, m) = -sys.Jc.transpose();
    KKT.block(n, 0, m, n) = sys.Jc;

    VectorXd rhs(n + m);
    rhs.head(n) = tau + f_ext - sys.h;
    rhs.tail(m).setZero();  // ac_des - Jc_dot * qd, here assumed zero

    VectorXd sol = KKT.fullPivLu().solve(rhs);
    qddot_out = sol.head(n);
    lambda_out = sol.tail(m);
}

int main() {
    VectorXd q(2), qd(2), tau(2), f_ext(2);
    q.setZero();
    qd.setZero();
    tau.setZero();
    f_ext.setZero();

    ContactSystem sys = buildContactSystem(q, qd);
    VectorXd qddot(2), lambda(1);
    solveContactDynamics(sys, tau, f_ext, qd, qddot, lambda);

    std::cout << "qddot = " << qddot.transpose() << std::endl;
    std::cout << "lambda = " << lambda.transpose() << std::endl;
    return 0;
}
      
