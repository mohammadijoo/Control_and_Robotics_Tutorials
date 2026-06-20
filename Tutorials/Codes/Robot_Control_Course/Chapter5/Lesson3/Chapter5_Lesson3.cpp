
#include <Eigen/Dense>

class SaturatedPDController {
public:
    SaturatedPDController(const Eigen::VectorXd& Kp,
                          const Eigen::VectorXd& Kd,
                          const Eigen::VectorXd& tau_min,
                          const Eigen::VectorXd& tau_max)
        : Kp_(Kp), Kd_(Kd), tau_min_(tau_min), tau_max_(tau_max) {}

    Eigen::VectorXd computeTau(const Eigen::VectorXd& q,
                               const Eigen::VectorXd& qd,
                               const Eigen::VectorXd& q_ref,
                               const Eigen::VectorXd& qd_ref) const
    {
        Eigen::VectorXd e  = q  - q_ref;
        Eigen::VectorXd ed = qd - qd_ref;
        Eigen::VectorXd tau_nom = -Kp_.cwiseProduct(e)
                                  -Kd_.cwiseProduct(ed);
        return saturate(tau_nom);
    }

private:
    Eigen::VectorXd saturate(const Eigen::VectorXd& tau_nom) const
    {
        Eigen::VectorXd tau = tau_nom;
        for (int i = 0; i < tau.size(); ++i) {
            if (tau(i) > tau_max_(i)) tau(i) = tau_max_(i);
            if (tau(i) < tau_min_(i)) tau(i) = tau_min_(i);
        }
        return tau;
    }

    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd tau_min_, tau_max_;
};

// In a dynamics loop (e.g., using Pinocchio):
// - compute M(q), b(q,qd)
// - compute tau_nom from controller
// - apply controller.computeTau(...)
// - integrate q, qd forward in time
