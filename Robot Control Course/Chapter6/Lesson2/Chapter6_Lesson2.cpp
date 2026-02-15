
#include <Eigen/Dense>

class RobotModel {
public:
    // User-provided implementations
    Eigen::Vector3d forwardKinematics(const Eigen::VectorXd &q) const;
    Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian(const Eigen::VectorXd &q) const;
    Eigen::VectorXd inverseDynamics(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &qdot,
                                    const Eigen::VectorXd &qddot) const;
    Eigen::Vector3d forceSensor() const;
};

class HybridPositionForceController {
public:
    HybridPositionForceController(const RobotModel *robot,
                                  const Eigen::Vector3d &contactNormalWorld)
        : robot_(robot), kf_(50.0)
    {
        n_ = contactNormalWorld.normalized();
        S_f_ = n_ * n_.transpose();
        S_p_ = Eigen::Matrix3d::Identity() - S_f_;

        Kp_tan_ = 200.0 * Eigen::Matrix3d::Identity();
        Kd_tan_ = 40.0 * Eigen::Matrix3d::Identity();
    }

    Eigen::VectorXd computeTorque(const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &qdot,
                                  const Eigen::Vector3d &x_d,
                                  const Eigen::Vector3d &xdot_d,
                                  double Fd_n)
    {
        // Forward kinematics and Jacobian
        Eigen::Vector3d x = robot_->forwardKinematics(q);
        Eigen::Matrix<double, 3, Eigen::Dynamic> J = robot_->jacobian(q);
        Eigen::Vector3d xdot = J * qdot;

        // Project desired position onto tangent plane
        Eigen::Vector3d delta = x_d - x;
        Eigen::Vector3d normal_component = (n_.dot(delta)) * n_;
        Eigen::Vector3d x_d_proj = x_d - normal_component;

        // Position errors in tangent space
        Eigen::Vector3d e_p = S_p_ * (x_d_proj - x);
        Eigen::Vector3d edot_p = S_p_ * (xdot_d - xdot);

        Eigen::Vector3d F_t_cmd = S_p_ * (Kp_tan_ * e_p + Kd_tan_ * edot_p);

        // Force loop (normal direction)
        Eigen::Vector3d F_meas = robot_->forceSensor();
        double F_n = n_.dot(F_meas);
        double e_f = Fd_n - F_n;

        double F_n_cmd = Fd_n + kf_ * e_f;
        Eigen::Vector3d F_n_vec = F_n_cmd * n_;

        // Total wrench
        Eigen::Vector3d F_cmd = F_t_cmd + F_n_vec;

        // Map to joint torques and add bias
        Eigen::VectorXd tau = J.transpose() * F_cmd;
        Eigen::VectorXd tau_bias = robot_->inverseDynamics(
            q, qdot, Eigen::VectorXd::Zero(q.size())
        );
        return tau + tau_bias;
    }

private:
    const RobotModel *robot_;
    Eigen::Vector3d n_;
    Eigen::Matrix3d S_p_, S_f_;
    Eigen::Matrix3d Kp_tan_, Kd_tan_;
    double kf_;
};
