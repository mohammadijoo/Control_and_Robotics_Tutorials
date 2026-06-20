#include <vector>
#include <Eigen/Dense>

struct TactileContact {
    Eigen::MatrixXd positions;  // K x 3
    Eigen::VectorXd pressures;  // K
    Eigen::VectorXd areas;      // K
    Eigen::Vector3d c_des;
    Eigen::Vector3d n_des;
    double mu;
};

static inline void center_of_pressure(
    const TactileContact& c,
    double& F_n,
    Eigen::Vector3d& c_est)
{
    Eigen::VectorXd w = c.pressures.cwiseProduct(c.areas);
    F_n = w.sum() + 1e-9;
    Eigen::Vector3d num = c.positions.transpose() * w;
    c_est = num / F_n;
}

static inline double slip_indicator(double F_n,
                                    double f_t_norm,
                                    double mu)
{
    return f_t_norm / (mu * F_n + 1e-9) - 1.0;
}

// User-provided Jacobian callbacks
typedef Eigen::MatrixXd (*JacobianFn)(const Eigen::VectorXd& q, int contact_index);

Eigen::VectorXd tactileGraspRefineStep(
    const Eigen::VectorXd& q,
    const std::vector<TactileContact>& contacts,
    JacobianFn Jc_fn,
    JacobianFn Jn_fn,
    double w_slip  = 1.0,
    double w_pos   = 1.0,
    double w_align = 0.1,
    double step_size = 1e-2,
    const Eigen::VectorXd* q_min = nullptr,
    const Eigen::VectorXd* q_max = nullptr)
{
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());

    for (std::size_t j = 0; j < contacts.size(); ++j) {
        const auto& c = contacts[j];
        double F_n;
        Eigen::Vector3d c_est;
        center_of_pressure(c, F_n, c_est);

        Eigen::Vector3d pos_err = c_est - c.c_des;
        double f_t_norm_est = pos_err.norm();

        double h = slip_indicator(F_n, f_t_norm_est, c.mu);
        double slip_penalty = std::max(0.0, h);

        Eigen::MatrixXd Jc = Jc_fn(q, static_cast<int>(j)); // 3 x n
        Eigen::MatrixXd Jn = Jn_fn(q, static_cast<int>(j)); // 3 x n

        Eigen::VectorXd grad_pos =
            2.0 * Jc.transpose() * pos_err;

        Eigen::Vector3d n_est = Eigen::Vector3d::Zero(); // placeholder
        Eigen::Vector3d n_err = n_est - c.n_des;
        Eigen::VectorXd grad_align =
            2.0 * Jn.transpose() * n_err;

        Eigen::VectorXd grad_slip = Eigen::VectorXd::Zero(q.size());
        if (slip_penalty > 0.0) {
            if (pos_err.norm() > 1e-6) {
                Eigen::Vector3d dir_vec = pos_err.normalized();
                Eigen::VectorXd dh_dq =
                    Jc.transpose() * dir_vec / (c.mu * F_n + 1e-9);
                grad_slip = 2.0 * slip_penalty * dh_dq;
            }
        }

        grad += w_pos * grad_pos
              + w_align * grad_align
              + w_slip * grad_slip;
    }

    Eigen::VectorXd dq = -step_size * grad;
    Eigen::VectorXd q_new = q + dq;

    if (q_min) {
        q_new = q_new.cwiseMax(*q_min);
    }
    if (q_max) {
        q_new = q_new.cwiseMin(*q_max);
    }
    return q_new;
}
      
