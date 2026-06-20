
#include <Eigen/Dense>
#include <vector>
#include <cmath>

struct RBFResidual {
    // phi(z) = exp(-gamma * ||z - c_j||^2) for j = 1..M
    std::vector<Eigen::VectorXd> centers;
    Eigen::VectorXd weights;
    double gamma;

    double predict(const Eigen::VectorXd& z) const {
        const int M = static_cast<int>(centers.size());
        Eigen::VectorXd phi(M);
        for (int j = 0; j < M; ++j) {
            double r2 = (z - centers[j]).squaredNorm();
            phi(j) = std::exp(-gamma * r2);
        }
        return weights.dot(phi);
    }
};

double nominal_torque(double q, double qd, double qd_des,
                      double qdd_des, double Kp, double Kd,
                      double m_nom, double b_nom, double g_nom)
{
    double e = q - qd_des;
    double edot = qd - qd_des; // qd_des is dot(q_d)
    double v = qdd_des - Kd * edot - Kp * e;
    double tau_nom = m_nom * v + b_nom * qd + g_nom * std::sin(q);
    return tau_nom;
}

double saturate(double x, double r_max) {
    if (x > r_max) return r_max;
    if (x < -r_max) return -r_max;
    return x;
}

// In the control loop (pseudo-code):
// 1. Build feature vector z = [q, qd, qd_des, qdd_des]^T
// 2. Compute tau_nom
// 3. Compute residual r_hat = saturate(rbf.predict(z), r_max)
// 4. Apply tau = tau_nom + r_hat
void controller_step(double q, double qd,
                     double qd_des, double qdd_des,
                     const RBFResidual& rbf,
                     double Kp, double Kd,
                     double m_nom, double b_nom, double g_nom,
                     double r_max,
                     double& tau_out)
{
    Eigen::VectorXd z(4);
    z << q, qd, qd_des, qdd_des;

    double tau_nom = nominal_torque(q, qd, qd_des, qdd_des,
                                    Kp, Kd, m_nom, b_nom, g_nom);
    double r_hat = saturate(rbf.predict(z), r_max);
    tau_out = tau_nom + r_hat;
}
