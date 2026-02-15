
#include <iostream>
#include <cmath>

struct State {
    double q;
    double dq;
    double d_hat;
};

const double m = 1.0;
const double l = 0.5;
const double g = 9.81;
const double I = m * l * l;

// control gains
const double lam = 4.0;
const double k_s = 5.0;
const double phi = 0.1;
const double alpha = 30.0;

double q_d(double t) {
    return 0.5 * std::sin(t);
}

double dq_d(double t) {
    return 0.5 * std::cos(t);
}

double ddq_d(double t) {
    return -0.5 * std::sin(t);
}

double disturbance(double t, double q, double dq) {
    // simple bounded disturbance
    double sign_dq = 0.0;
    if (dq > 0.0) sign_dq = 1.0;
    if (dq < 0.0) sign_dq = -1.0;
    return 1.5 * std::sin(3.0 * t) + 0.5 * sign_dq;
}

double sat(double s, double phi) {
    double sigma = s / phi;
    if (sigma > 1.0) return 1.0;
    if (sigma < -1.0) return -1.0;
    return sigma;
}

State dynamics(double t, const State& x) {
    State dx;
    double q = x.q;
    double dq = x.dq;
    double d_hat = x.d_hat;

    double e = q - q_d(t);
    double de = dq - dq_d(t);
    double s = de + lam * e;

    double g_term = m * g * l * std::sin(q);

    double tau_nom = I * (ddq_d(t) - lam * de) + g_term;
    double tau_rob = -k_s * sat(s, phi);
    double tau = tau_nom + tau_rob - d_hat;

    double d = disturbance(t, q, dq);
    double ddq = (tau + d - g_term) / I;

    double q_ddot_est = ddq;
    double d_hat_dot = -alpha * d_hat + alpha * (I * q_ddot_est + g_term - tau);

    dx.q = dq;
    dx.dq = ddq;
    dx.d_hat = d_hat_dot;
    return dx;
}

int main() {
    double t = 0.0;
    const double tf = 20.0;
    const double dt = 0.001;

    State x{0.0, 0.0, 0.0};

    while (t < tf) {
        State k1 = dynamics(t, x);
        // simple explicit Euler for illustration
        x.q += dt * k1.q;
        x.dq += dt * k1.dq;
        x.d_hat += dt * k1.d_hat;

        t += dt;

        if (std::fmod(t, 0.5) < dt) {
            double e = x.q - q_d(t);
            std::cout << t << " " << x.q << " " << e << "\n";
        }
    }

    return 0;
}
