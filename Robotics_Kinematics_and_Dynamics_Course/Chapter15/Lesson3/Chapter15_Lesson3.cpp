#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Solve KKT system:
// [ M  J^T ] [ qdd ]   = [ tau - h ]
// [ J   0  ] [ lam ]     [   -gamma ]
// where gamma = Jdot * qdot for holonomic constraints.
struct KKTResult {
    VectorXd qdd;
    VectorXd lambda;
};

KKTResult solveKKT(const MatrixXd& M,
                   const MatrixXd& J,
                   const VectorXd& tau_minus_h,
                   const VectorXd& gamma)
{
    int n = static_cast<int>(M.rows());
    int m = static_cast<int>(J.rows());

    MatrixXd K(n + m, n + m);
    K.setZero();
    K.block(0, 0, n, n) = M;
    K.block(0, n, n, m) = J.transpose();
    K.block(n, 0, m, n) = J;

    VectorXd rhs(n + m);
    rhs.segment(0, n) = tau_minus_h;
    rhs.segment(n, m) = -gamma;

    VectorXd sol = K.fullPivLu().solve(rhs);

    KKTResult res;
    res.qdd    = sol.segment(0, n);
    res.lambda = sol.segment(n, m);
    return res;
}

int main()
{
    // Example sizes for a 4-bar: n = 3, m = 2
    MatrixXd M = MatrixXd::Identity(3, 3);
    MatrixXd J(2, 3);
    J << 1.0, 0.0, 0.0,
          0.0, 1.0, 1.0;

    VectorXd tau_minus_h(3);
    tau_minus_h << 1.0, 0.0, 0.0;

    VectorXd gamma(2);
    gamma.setZero();

    KKTResult res = solveKKT(M, J, tau_minus_h, gamma);

    std::cout << "qdd = " << res.qdd.transpose() << std::endl;
    std::cout << "lambda = " << res.lambda.transpose() << std::endl;

    return 0;
}
      
