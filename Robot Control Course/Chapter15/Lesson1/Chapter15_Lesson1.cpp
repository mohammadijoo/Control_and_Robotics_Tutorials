
#include <iostream>
#include <Eigen/Dense>

// Simple 1-DOF joint; in practice you may integrate with ROS control and Pinocchio/RBDL
double M(double q) { return 1.2; }
double C(double q, double qd) { return 0.0; }
double g(double q) { return 0.0; }

// Learned friction model: tau_f_hat(qd) = w0 * qd + w1 * sign(qd)
struct LearnedFriction {
    Eigen::Vector2d w;
    LearnedFriction() {
        w << 0.25, 0.08; // trained offline
    }
    double predict(double qd) const {
        double s = (qd > 0.0) ? 1.0 : (qd < 0.0 ? -1.0 : 0.0);
        Eigen::Vector2d phi(qd, s);
        return w.dot(phi);
    }
};

double jointController(double q, double qd,
                       double q_ref, double qd_ref, double qdd_ref,
                       const LearnedFriction& model)
{
    const double Kp = 50.0;
    const double Kd = 10.0;
    double q_tilde = q - q_ref;
    double qd_tilde = qd - qd_ref;
    double v = qdd_ref - Kd * qd_tilde - Kp * q_tilde;
    double tau_nom = M(q) * v + C(q, qd) * qd + g(q);
    double tau_learn = model.predict(qd);
    return tau_nom + tau_learn;
}
