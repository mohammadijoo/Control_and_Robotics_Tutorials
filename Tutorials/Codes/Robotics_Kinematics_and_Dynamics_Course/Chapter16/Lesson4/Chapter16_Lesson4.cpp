#include <Eigen/Dense>

// Assume you have functions that wrap an RBDL or Pinocchio model:
void computeDynamics(const Eigen::VectorXd& q,
                     const Eigen::VectorXd& qdot,
                     Eigen::MatrixXd& M,
                     Eigen::VectorXd& h); // h = C(q,qdot) qdot + g(q)

void computeConstraints(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& qdot,
                        Eigen::MatrixXd& Jc,
                        Eigen::VectorXd& Jc_dot_qdot);

Eigen::MatrixXd Sa(); // selection matrix for actuated joints

Eigen::VectorXd tau_a(double t,
                      const Eigen::VectorXd& q,
                      const Eigen::VectorXd& qdot);

Eigen::VectorXd rhs_closed_chain(double t,
                                 const Eigen::VectorXd& x)
{
    int n = static_cast<int>(x.size()) / 2;
    Eigen::VectorXd q   = x.head(n);
    Eigen::VectorXd qdot = x.tail(n);

    Eigen::MatrixXd M(n, n);
    Eigen::VectorXd h(n);
    computeDynamics(q, qdot, M, h);

    Eigen::MatrixXd Jc;
    Eigen::VectorXd Jc_dot_qdot;
    computeConstraints(q, qdot, Jc, Jc_dot_qdot);

    Eigen::MatrixXd S = Sa();
    Eigen::VectorXd tau = tau_a(t, q, qdot);

    int m = static_cast<int>(Jc.rows());

    Eigen::MatrixXd K(n + m, n + m);
    K.setZero();
    K.block(0, 0, n, n)       = M;
    K.block(0, n, n, m)       = -Jc.transpose();
    K.block(n, 0, m, n)       = Jc;
    // lower-right m x m block stays zero

    Eigen::VectorXd b(n + m);
    b.head(n)   = S.transpose() * tau - h;
    b.tail(m)   = -Jc_dot_qdot;

    Eigen::VectorXd sol = K.fullPivLu().solve(b);
    Eigen::VectorXd qddot = sol.head(n);
    // Eigen::VectorXd lambda = sol.tail(m);

    Eigen::VectorXd xdot(x.size());
    xdot.head(n) = qdot;
    xdot.tail(n) = qddot;
    return xdot;
}
      
