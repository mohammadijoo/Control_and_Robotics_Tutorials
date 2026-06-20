
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct ResidualModel {
    MatrixXd W;
    double eta;

    ResidualModel(int out_dim, int in_dim, double eta_)
        : W(MatrixXd::Zero(out_dim, in_dim)), eta(eta_) {}

    VectorXd predict(const VectorXd &phi) const {
        return W * phi;
    }

    void update(const VectorXd &phi, const VectorXd &target) {
        VectorXd err = predict(phi) - target;
        // SGD step: W = W - eta * err * phi^T
        W.noalias() -= eta * (err * phi.transpose());
    }
};

// Nominal computed-torque (here just as an interface)
VectorXd computeNominalTorque(const VectorXd &q,
                              const VectorXd &dq,
                              const VectorXd &qd,
                              const VectorXd &dqd,
                              const VectorXd &ddqd);

VectorXd buildFeatures(const VectorXd &q,
                       const VectorXd &dq,
                       const VectorXd &qd,
                       const VectorXd &dqd,
                       const VectorXd &ddqd) {
    // Example: stack [q; dq; qd; dqd; ddqd]
    int n = q.size();
    VectorXd phi(5 * n);
    phi.segment(0, n)      = q;
    phi.segment(n, n)      = dq;
    phi.segment(2 * n, n)  = qd;
    phi.segment(3 * n, n)  = dqd;
    phi.segment(4 * n, n)  = ddqd;
    return phi;
}

// One control step with residual augmentation and optional online learning
VectorXd controlStepResidual(ResidualModel &residual,
                             const VectorXd &q,
                             const VectorXd &dq,
                             const VectorXd &qd,
                             const VectorXd &dqd,
                             const VectorXd &ddqd,
                             const VectorXd &tau_meas,
                             bool do_update) {
    VectorXd phi = buildFeatures(q, dq, qd, dqd, ddqd);
    VectorXd tau_nom = computeNominalTorque(q, dq, qd, dqd, ddqd);
    VectorXd tau_res = residual.predict(phi);
    VectorXd tau_cmd = tau_nom + tau_res;

    if (do_update) {
        // Residual target: tau_meas - tau_nom
        VectorXd target = tau_meas - tau_nom;
        residual.update(phi, target);
    }

    return tau_cmd;
}
