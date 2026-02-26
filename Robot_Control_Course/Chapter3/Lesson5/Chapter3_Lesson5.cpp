
#include <iostream>
#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Matrix2d;

const double m1 = 1.0, m2 = 1.0;
const double l1 = 1.0, l2 = 1.0;
const double lc1 = 0.5, lc2 = 0.5;
const double I1 = 0.1, I2 = 0.1;
const double g = 9.81;

Matrix2d M_matrix(const Vector2d& q) {
    double q1 = q(0), q2 = q(1);
    double c2 = std::cos(q2);
    double m11 = I1 + I2 + m1*lc1*lc1
                 + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*c2);
    double m12 = I2 + m2*(lc2*lc2 + l1*lc2*c2);
    double m22 = I2 + m2*lc2*lc2;
    Matrix2d M;
    M << m11, m12,
          m12, m22;
    return M;
}

Matrix2d C_matrix(const Vector2d& q, const Vector2d& qdot) {
    double q2 = q(1);
    double q1dot = qdot(0), q2dot = qdot(1);
    double s2 = std::sin(q2);
    double h = -m2*l1*lc2*s2;
    Matrix2d C;
    C << h*q2dot, h*(q1dot + q2dot),
          -h*q1dot, 0.0;
    return C;
}

Vector2d g_vector(const Vector2d& q) {
    double q1 = q(0), q2 = q(1);
    double g1 = (m1*lc1 + m2*l1)*g*std::cos(q1)
                + m2*lc2*g*std::cos(q1 + q2);
    double g2 = m2*lc2*g*std::cos(q1 + q2);
    return Vector2d(g1, g2);
}

Vector2d qd(double t) {
    return Vector2d(0.5*std::sin(0.5*t),
                    0.5*std::cos(0.5*t));
}

Vector2d qd_dot(double t) {
    return Vector2d(0.25*std::cos(0.5*t),
                    -0.25*std::sin(0.5*t));
}

Vector2d qd_ddot(double t) {
    return Vector2d(-0.125*std::sin(0.5*t),
                    -0.125*std::cos(0.5*t));
}

Matrix2d Kp = (Matrix2d() << 100.0, 0.0, 0.0, 80.0).finished();
Matrix2d Kd = (Matrix2d() << 20.0, 0.0, 0.0, 16.0).finished();

Vector2d controller_PD(const Vector2d& q,
                       const Vector2d& qdot,
                       double t) {
    Vector2d e = qd(t) - q;
    Vector2d edot = qd_dot(t) - qdot;
    return Kp*e + Kd*edot;
}

Vector2d controller_CT(const Vector2d& q,
                       const Vector2d& qdot,
                       double t) {
    Vector2d e = qd(t) - q;
    Vector2d edot = qd_dot(t) - qdot;
    Vector2d v = qd_ddot(t) + Kd*edot + Kp*e;
    Matrix2d M = M_matrix(q);
    Matrix2d C = C_matrix(q, qdot);
    Vector2d g_vec = g_vector(q);
    return M*v + C*qdot + g_vec;
}

struct Metrics {
    double Je;
    double Jtau;
    double e_max;
};

Metrics simulate(Vector2d (*controller)(const Vector2d&, const Vector2d&, double),
                 double T, double dt) {
    int steps = static_cast<int>(T / dt);
    Vector2d q(0.0, 0.0);
    Vector2d qdot(0.0, 0.0);

    double Je = 0.0, Jtau = 0.0, e_max = 0.0;

    for (int k = 0; k < steps; ++k) {
        double t = k * dt;
        Vector2d e = qd(t) - q;
        Vector2d edot = qd_dot(t) - qdot;
        Vector2d tau = controller(q, qdot, t);

        Matrix2d M = M_matrix(q);
        Matrix2d C = C_matrix(q, qdot);
        Vector2d g_vec = g_vector(q);

        Vector2d qddot = M.ldlt().solve(tau - C*qdot - g_vec);

        qdot += dt * qddot;
        q += dt * qdot;

        Je += e.dot(e) * dt;
        Jtau += tau.dot(tau) * dt;
        double e_norm = e.norm();
        if (e_norm > e_max) e_max = e_norm;
    }
    return {Je, Jtau, e_max};
}

int main() {
    double T = 10.0, dt = 0.001;
    Metrics mPD = simulate(controller_PD, T, dt);
    Metrics mCT = simulate(controller_CT, T, dt);

    std::cout << "PD: J_e=" << mPD.Je
              << " J_tau=" << mPD.Jtau
              << " e_max=" << mPD.e_max << std::endl;

    std::cout << "CT: J_e=" << mCT.Je
              << " J_tau=" << mCT.Jtau
              << " e_max=" << mCT.e_max << std::endl;
    return 0;
}
