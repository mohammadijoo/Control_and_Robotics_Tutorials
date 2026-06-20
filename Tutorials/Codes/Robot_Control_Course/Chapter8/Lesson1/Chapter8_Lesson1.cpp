
#include <iostream>
#include <Eigen/Dense>

struct Params {
    double m_true{2.0};
    double b_true{0.4};
    double g0_true{5.0};
    double m_hat{1.0};
    double b_hat{0.2};
    double g0_hat{3.0};
};

void desired_trajectory(double t, double& qd, double& dqd, double& ddqd) {
    qd = 0.5 * std::sin(0.5 * t);
    dqd = 0.25 * std::cos(0.5 * t);
    ddqd = -0.125 * std::sin(0.5 * t);
}

double control_law(double t, double q, double dq,
                   const Params& p,
                   double k_p, double k_d,
                   Eigen::Vector3d& Y, double& tau_hat) {
    double qd, dqd, ddqd;
    desired_trajectory(t, qd, dqd, ddqd);
    double e = q - qd;
    double de = dq - dqd;
    double ddq_r = ddqd - k_d * de - k_p * e;
    double dq_r = dqd - k_p * e;

    Y << ddq_r, dq_r, std::sin(q);
    Eigen::Vector3d theta_hat(p.m_hat, p.b_hat, p.g0_hat);
    tau_hat = Y.dot(theta_hat);

    double tau = tau_hat; // plus optional extra feedback terms
    return tau;
}

int main() {
    Params p;
    double k_p = 20.0, k_d = 8.0;
    double dt = 0.001, T = 5.0;
    int steps = static_cast<int>(T / dt);

    double q = 0.0, dq = 0.0;
    Eigen::Vector3d Y;
    for (int k = 0; k < steps; ++k) {
        double t = k * dt;
        double tau_hat = 0.0;
        double tau = control_law(t, q, dq, p, k_p, k_d, Y, tau_hat);

        // Gradient-like parameter update (illustrative)
        double error = tau - tau_hat;
        double gamma = 0.1;
        Eigen::Vector3d grad = -Y * error;
        p.m_hat -= gamma * grad(0);
        p.b_hat -= gamma * grad(1);
        p.g0_hat -= gamma * grad(2);

        // Plant integration (semi-implicit Euler)
        double ddq = (tau - p.b_true * dq - p.g0_true * std::sin(q)) / p.m_true;
        dq += ddq * dt;
        q += dq * dt;

        if (k % 1000 == 0) {
            std::cout << "t=" << t
                      << " q=" << q
                      << " m_hat=" << p.m_hat << std::endl;
        }
    }
    return 0;
}
