#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kinetic energy T(q, dq) = 0.5 * dq^T M(q) dq
double kineticEnergy(const MatrixXd& Mq, const VectorXd& dq) {
    return 0.5 * dq.transpose() * Mq * dq;
}

// Example: call kineticEnergy inside a dynamics routine
int main() {
    const int n = 2;
    MatrixXd Mq(n, n);
    VectorXd dq(n);

    // Example numeric metric (e.g. evaluated at some configuration q)
    Mq << 2.0, 0.5,
          0.5, 1.0;

    dq << 0.3, -0.1;

    double T = kineticEnergy(Mq, dq);
    // T is the Riemannian energy 0.5 * g_q(dq, dq)
    return 0;
}
      
