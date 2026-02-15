
#include "Eigen/Dense"

class RobotModel {
public:
    int numParams() const;
    Eigen::MatrixXd regressor(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& dq,
                              const Eigen::VectorXd& qr,
                              const Eigen::VectorXd& dqr,
                              const Eigen::VectorXd& ddqr) const;
};

class AdaptiveCTControllerCpp {
public:
    AdaptiveCTControllerCpp(int n_dof,
                            const Eigen::MatrixXd& Kd,
                            const Eigen::MatrixXd& Lambda,
                            const Eigen::MatrixXd& Gamma_d,
                            const RobotModel& model)
        : n(n_dof),
          Kd_(Kd),
          Lambda_(Lambda),
          Gamma_d_(Gamma_d),
          robot_(model)
    {
        theta_hat_.setZero(robot_.numParams());
    }

    void step(const Eigen::VectorXd& q,
              const Eigen::VectorXd& dq,
              const Eigen::VectorXd& qd,
              const Eigen::VectorXd& dqd,
              const Eigen::VectorXd& ddqd,
              Eigen::VectorXd& tau_out)
    {
        Eigen::VectorXd e  = q - qd;
        Eigen::VectorXd de = dq - dqd;
        Eigen::VectorXd s  = de + Lambda_ * e;

        Eigen::VectorXd dqr  = dqd - Lambda_ * e;
        Eigen::VectorXd ddqr = ddqd - Lambda_ * de;

        Eigen::MatrixXd Y = robot_.regressor(q, dq, dqr, ddqr);

        tau_out = Y * theta_hat_ - Kd_ * s;

        Eigen::VectorXd grad = Y.transpose() * s;
        theta_hat_ = theta_hat_ - Gamma_d_ * grad;
    }

    const Eigen::VectorXd& thetaHat() const { return theta_hat_; }

private:
    int n;
    Eigen::MatrixXd Kd_, Lambda_, Gamma_d_;
    Eigen::VectorXd theta_hat_;
    const RobotModel& robot_;
};
