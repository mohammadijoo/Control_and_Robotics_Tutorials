
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct SmcGains {
    MatrixXd Lambda;  // n x n
    MatrixXd K;       // n x n (diagonal)
    VectorXd phi;     // n
};

VectorXd smcTorque(
    const VectorXd& q,
    const VectorXd& dq,
    const VectorXd& q_d,
    const VectorXd& dq_d,
    const VectorXd& ddq_d,
    const SmcGains& gains
) {
    const MatrixXd& Lambda = gains.Lambda;
    const MatrixXd& K      = gains.K;
    const VectorXd& phi    = gains.phi;

    // Tracking error and sliding variable
    VectorXd e     = q - q_d;
    VectorXd e_dot = dq - dq_d;
    VectorXd s     = e_dot + Lambda * e;

    // Robot dynamics (user must implement or wrap a robotics library)
    MatrixXd M = M_robot(q);       // n x n
    MatrixXd C = C_robot(q, dq);   // n x n
    VectorXd g = g_robot(q);       // n

    VectorXd tau_eq = M * (ddq_d - Lambda * e_dot) + C * dq + g;

    // Smooth saturation via tanh
    VectorXd phi_safe = phi.cwiseMax(1e-6);
    VectorXd sat_arg  = s.cwiseQuotient(phi_safe);
    VectorXd sat_val  = sat_arg.array().tanh();

    VectorXd tau_sw = -K * sat_val;
    VectorXd tau    = tau_eq + tau_sw;
    return tau;
}
