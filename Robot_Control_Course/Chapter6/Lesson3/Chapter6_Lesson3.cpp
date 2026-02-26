
#include <iostream>
#include <cmath>

struct JointImpedanceController {
    double J;  // inertia
    double K;  // stiffness
    double D;  // damping

    JointImpedanceController(double J_, double K_, double D_)
        : J(J_), K(K_), D(D_) {}

    // Compute motor torque given states and desired trajectory
    double computeTorque(double q, double qd,
                         double q_ref, double qd_ref, double qdd_ref,
                         double tau_ext) {
        double e = q - q_ref;
        double ed = qd - qd_ref;
        double tau_m = J * qdd_ref - K * e - D * ed;
        // total torque applied to joint (motor only)
        return tau_m;
    }
};

int main() {
    JointImpedanceController ctrl(0.05, 10.0, 2.0);

    double q = 0.0;
    double qd = 0.0;
    double dt = 0.001;
    double T = 1.0;
    int N = static_cast<int>(T / dt);

    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        double q_ref = (t >= 0.1) ? 0.5 : 0.0;
        double qd_ref = 0.0;
        double qdd_ref = 0.0;

        double tau_ext = (t >= 0.3 && t <= 0.35) ? 0.2 : 0.0;

        double tau_m = ctrl.computeTorque(q, qd, q_ref, qd_ref, qdd_ref, tau_ext);

        // Plant integration: J * qdd = tau_m + tau_ext
        double qdd = (tau_m + tau_ext) / ctrl.J;

        qd += dt * qdd;
        q += dt * qd;
    }

    std::cout << "Final q = " << q << std::endl;
    return 0;
}
