#include <Eigen/Dense>
#include <functional>

// Type aliases
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

// User-supplied dynamics (to be implemented or connected to a robotics library)
Matrix M(const Vector& q);
Matrix C(const Vector& q, const Vector& dq);
Vector g_vec(const Vector& q);
double potentialEnergy(const Vector& q);

double kineticEnergy(const Vector& q, const Vector& dq) {
    Matrix Mq = M(q);
    return 0.5 * dq.transpose() * Mq * dq;
}

double totalEnergy(const Vector& q, const Vector& dq) {
    return kineticEnergy(q, dq) + potentialEnergy(q);
}

Matrix directionalMDot(const Vector& q, const Vector& dq, double eps = 1e-6) {
    Vector q_plus = q + eps * dq;
    Vector q_minus = q - eps * dq;
    Matrix M_plus = M(q_plus);
    Matrix M_minus = M(q_minus);
    return (M_plus - M_minus) / (2.0 * eps);
}

double scalarSigma(const Vector& q, const Vector& dq) {
    Matrix Mdot = directionalMDot(q, dq);
    Matrix Cmat = C(q, dq);
    Matrix middle = 0.5 * Mdot - Cmat;
    return dq.transpose() * middle * dq;
}

Vector dynamicsRhs(const Vector& q, const Vector& dq, const Vector& tau) {
    Matrix Mq = M(q);
    Matrix Cq = C(q, dq);
    Vector gq = g_vec(q);
    Vector rhs = tau - Cq * dq - gq;
    // Solve M qdd = rhs
    return Mq.ldlt().solve(rhs);
}

void stepEuler(Vector& q, Vector& dq,
               const std::function<Vector(double, const Vector&, const Vector&)>& tauFunc,
               double t, double dt) {
    Vector tau = tauFunc(t, q, dq);
    Vector qdd = dynamicsRhs(q, dq, tau);
    q += dt * dq;
    dq += dt * qdd;
}
      
