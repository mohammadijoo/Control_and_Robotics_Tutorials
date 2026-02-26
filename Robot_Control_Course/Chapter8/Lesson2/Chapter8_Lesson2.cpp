
#include <Eigen/Dense>

class AdaptiveCT1DOF {
public:
    AdaptiveCT1DOF()
        : Lambda_(10.0), kD_(5.0)
    {
        theta_hat_.setZero();
        theta_hat_ << 0.5, 0.1, 0.5; // initial guess
        Gamma_.setZero();
        Gamma_.diagonal() << 5.0, 1.0, 1.0;
    }

    double step(double q, double qd,
                double q_d, double qd_d, double qdd_d,
                double dt)
    {
        double tilde_q  = q - q_d;
        double tilde_qd = qd - qd_d;

        double qrd  = qd_d - Lambda_ * tilde_q;
        double qrdd = qdd_d - Lambda_ * tilde_qd;
        double s    = qd - qrd;

        Eigen::RowVector3d Y;
        Y << qrdd, qrd, std::cos(q);

        double tau = Y * theta_hat_ + kD_ * s;

        Eigen::Vector3d theta_dot = -Gamma_ * (Y.transpose() * s);
        theta_hat_ += theta_dot * dt;

        return tau;
    }

    const Eigen::Vector3d& theta_hat() const { return theta_hat_; }

private:
    double Lambda_;
    double kD_;
    Eigen::Vector3d theta_hat_;
    Eigen::Matrix3d Gamma_;
};
