
#include <Eigen/Dense>

class JointDOB {
public:
    JointDOB(double J_nom, double w_c, double dt)
        : J_nom_(J_nom), w_c_(w_c), dt_(dt),
          tau_u_hat_(0.0), q_prev_(0.0), qdot_prev_(0.0), initialized_(false) {}

    double update(double q, double qdot, double tau_c) {
        if (!initialized_) {
            q_prev_ = q;
            qdot_prev_ = qdot;
            initialized_ = true;
        }

        // Acceleration estimate
        double qddot = (qdot - qdot_prev_) / dt_;

        // Disturbance estimate update
        double rhs = -w_c_ * tau_u_hat_ + w_c_ * (J_nom_ * qddot - tau_c);
        tau_u_hat_ += dt_ * rhs;

        double tau = tau_c - tau_u_hat_;

        q_prev_ = q;
        qdot_prev_ = qdot;
        return tau;
    }

private:
    double J_nom_;
    double w_c_;
    double dt_;
    double tau_u_hat_;
    double q_prev_;
    double qdot_prev_;
    bool initialized_;
};

// Example vector extension (sketch):
// Eigen::VectorXd tau = tau_c - tau_u_hat;
// tau_u_hat += dt_ * (-w_c_ * tau_u_hat + w_c_ * (J_nom_.asDiagonal() * qddot - tau_c));
