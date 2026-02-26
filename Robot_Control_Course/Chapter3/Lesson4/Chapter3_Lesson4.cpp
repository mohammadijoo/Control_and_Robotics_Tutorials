
#include <Eigen/Dense>
#include <algorithm>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

struct RobotModel {
    Matrix M(const Vector& q) const;
    Matrix C(const Vector& q, const Vector& qdot) const;
    Vector g(const Vector& q) const;
};

class JointFilter {
public:
    JointFilter(int n, double Ts, double omega_c_vel, double omega_c_pos = -1.0)
        : n_(n), Ts_(Ts),
          q_prev_(Vector::Zero(n)),
          qf_(Vector::Zero(n)),
          qdot_est_(Vector::Zero(n)) {

        alpha_vel_ = 1.0 / (1.0 + omega_c_vel * Ts_);
        if (omega_c_pos > 0.0) {
            beta_pos_ = 1.0 / (1.0 + omega_c_pos * Ts_);
        } else {
            beta_pos_ = -1.0; // disabled
        }
    }

    void update(const Vector& q_meas) {
        if (beta_pos_ > 0.0) {
            qf_ = beta_pos_ * qf_ + (1.0 - beta_pos_) * q_meas;
        } else {
            qf_ = q_meas;
        }

        Vector dq_raw = (q_meas - q_prev_) / Ts_;
        qdot_est_ = alpha_vel_ * qdot_est_
                  + (1.0 - alpha_vel_) * dq_raw;

        q_prev_ = q_meas;
    }

    const Vector& q_filtered() const { return qf_; }
    const Vector& qdot_est() const { return qdot_est_; }

private:
    int n_;
    double Ts_;
    double alpha_vel_;
    double beta_pos_;
    Vector q_prev_;
    Vector qf_;
    Vector qdot_est_;
};

inline Vector saturate(const Vector& u,
                       const Vector& u_min,
                       const Vector& u_max) {
    Vector out = u;
    for (int i = 0; i < u.size(); ++i) {
        out[i] = std::min(std::max(out[i], u_min[i]), u_max[i]);
    }
    return out;
}

inline Vector rateLimit(const Vector& u_cmd,
                        const Vector& u_prev,
                        const Vector& du_min,
                        const Vector& du_max) {
    Vector out = u_prev;
    for (int i = 0; i < u_cmd.size(); ++i) {
        double du = u_cmd[i] - u_prev[i];
        du = std::min(std::max(du, du_min[i]), du_max[i]);
        out[i] += du;
    }
    return out;
}

class ComputedTorqueController {
public:
    ComputedTorqueController(const RobotModel* robot,
                             const Vector& Kp_diag,
                             const Vector& Kd_diag,
                             double Ts,
                             const Vector& tau_min,
                             const Vector& tau_max,
                             const Vector& dtau_min,
                             const Vector& dtau_max,
                             double omega_c_vel,
                             double omega_c_pos = -1.0)
        : robot_(robot),
          Kp_(Kp_diag.asDiagonal()),
          Kd_(Kd_diag.asDiagonal()),
          Ts_(Ts),
          tau_min_(tau_min),
          tau_max_(tau_max),
          dtau_min_(dtau_min),
          dtau_max_(dtau_max),
          filter_(tau_min.size(), Ts, omega_c_vel, omega_c_pos),
          tau_prev_(Vector::Zero(tau_min.size())) {}

    Vector step(const Vector& q_meas,
                const Vector& qd,
                const Vector& qd_dot,
                const Vector& qd_ddot) {
        // 1) Filtering
        filter_.update(q_meas);
        const Vector& qf = filter_.q_filtered();
        const Vector& qdot_est = filter_.qdot_est();

        // 2) Errors
        Vector e_q = qd - qf;
        Vector e_qdot = qd_dot - qdot_est;

        // 3) Dynamics
        Matrix M = robot_->M(qf);
        Matrix C = robot_->C(qf, qdot_est);
        Vector g = robot_->g(qf);

        Vector v = qd_ddot + Kd_ * e_qdot + Kp_ * e_q;
        Vector tau_pre = M * v + C * qdot_est + g;

        // 4) Rate limit and saturate
        Vector tau_rl = rateLimit(tau_pre, tau_prev_, dtau_min_, dtau_max_);
        Vector tau_sat = saturate(tau_rl, tau_min_, tau_max_);
        tau_prev_ = tau_sat;
        return tau_sat;
    }

private:
    const RobotModel* robot_;
    Matrix Kp_;
    Matrix Kd_;
    double Ts_;
    Vector tau_min_, tau_max_;
    Vector dtau_min_, dtau_max_;
    JointFilter filter_;
    Vector tau_prev_;
};
