
#include <Eigen/Dense>

double trackingCost(const Eigen::MatrixXd& q,
                    const Eigen::MatrixXd& qd,
                    const Eigen::MatrixXd& Q,
                    double dt)
{
    // q, qd: (N+1) x n
    const int Np1 = static_cast<int>(q.rows());
    double J = 0.0;
    for (int k = 0; k < Np1; ++k) {
        Eigen::VectorXd e = q.row(k).transpose() - qd.row(k).transpose();
        J += e.transpose() * Q * e;
    }
    return 0.5 * dt * J;
}

double effortCost(const Eigen::MatrixXd& tau,
                  const Eigen::MatrixXd& R,
                  double dt)
{
    // tau: N x n
    const int N = static_cast<int>(tau.rows());
    double J = 0.0;
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd t = tau.row(k).transpose();
        J += t.transpose() * R * t;
    }
    return 0.5 * dt * J;
}

double smoothnessCost(const Eigen::MatrixXd& tau,
                      const Eigen::MatrixXd& Su)
{
    const int N = static_cast<int>(tau.rows());
    if (N < 2) return 0.0;
    double J = 0.0;
    for (int k = 1; k < N; ++k) {
        Eigen::VectorXd dtau =
            tau.row(k).transpose() - tau.row(k - 1).transpose();
        J += dtau.transpose() * Su * dtau;
    }
    return 0.5 * J;
}

double totalCost(const Eigen::MatrixXd& q,
                 const Eigen::MatrixXd& qd,
                 const Eigen::MatrixXd& tau,
                 const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& Su,
                 double dt,
                 double lambdaTrack,
                 double lambdaEffort,
                 double lambdaSmooth)
{
    double Jtr = trackingCost(q, qd, Q, dt);
    double Jeff = effortCost(tau, R, dt);
    double Js = smoothnessCost(tau, Su);
    return lambdaTrack * Jtr
         + lambdaEffort * Jeff
         + lambdaSmooth * Js;
}
