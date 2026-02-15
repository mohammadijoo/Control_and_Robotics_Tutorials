#include <Eigen/Dense>

struct RobotState {
    Eigen::Vector3d ee_pos;
    Eigen::Vector3d goal;
    bool success;
};

double baseReward(const RobotState& s,
                  const RobotState& s_next,
                  const Eigen::VectorXd& action,
                  double ctrl_coeff = 0.01)
{
    double dist = (s.ee_pos - s.goal).norm();
    double r_task = -dist;
    double r_ctrl = -ctrl_coeff * action.squaredNorm();
    double r_term = s_next.success ? 1.0 : 0.0;
    return r_task + r_ctrl + r_term;
}

double potential(const RobotState& s, double alpha = 0.5)
{
    return -alpha * (s.ee_pos - s.goal).norm();
}

double shapedReward(const RobotState& s,
                    const RobotState& s_next,
                    const Eigen::VectorXd& action,
                    double gamma = 0.99,
                    double alpha = 0.5)
{
    double r_base = baseReward(s, s_next, action);
    double phi_s = potential(s, alpha);
    double phi_next = potential(s_next, alpha);
    double r_shape = gamma * phi_next - phi_s;
    return r_base + r_shape;
}
      
