#include <Eigen/Dense>

Eigen::VectorXd saturateTorque(
    const Eigen::VectorXd& tau_cmd,
    const Eigen::VectorXd& tau_min,
    const Eigen::VectorXd& tau_max)
{
    Eigen::VectorXd tau = tau_cmd;
    for (int i = 0; i < tau.size(); ++i) {
        if (tau(i) > tau_max(i)) {
            tau(i) = tau_max(i);
        } else if (tau(i) < tau_min(i)) {
            tau(i) = tau_min(i);
        }
    }
    return tau;
}

// Example: wrap inverse dynamics (not shown) with saturation
Eigen::VectorXd computeSaturatedTorque(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd_des,
    const Eigen::VectorXd& tau_min,
    const Eigen::VectorXd& tau_max)
{
    // Suppose invDynamics(q, qd, qdd_des) is provided by a dynamics library
    Eigen::VectorXd tau_cmd = invDynamics(q, qd, qdd_des);
    Eigen::VectorXd tau_act = saturateTorque(tau_cmd, tau_min, tau_max);
    return tau_act;
}
      
