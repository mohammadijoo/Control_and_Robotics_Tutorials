
#include <iostream>
#include <cmath>

struct PDGains {
    double Kp;
    double Kd;
};

double dampingRatioFromOvershoot(double Mp_star) {
    if (Mp_star <= 0.0 || Mp_star >= 1.0) {
        throw std::runtime_error("Mp_star must be in (0, 1).");
    }
    double logMp = std::log(Mp_star);
    return -logMp / std::sqrt(M_PI * M_PI + logMp * logMp);
}

PDGains computePDGains(double J, double B, double Mp_star, double Ts_star) {
    double zeta = dampingRatioFromOvershoot(Mp_star);
    double wn   = 4.0 / (zeta * Ts_star);
    PDGains g;
    g.Kp = J * wn * wn;
    g.Kd = 2.0 * J * zeta * wn - B;
    if (g.Kd < 0.0) g.Kd = 0.0;
    return g;
}

double pdTorque(double q, double qd, double dq, const PDGains& g) {
    double e  = q - qd;
    double de = dq; // assume qd is constant
    return -g.Kp * e - g.Kd * de;
}

int main() {
    double J = 0.5;
    double B = 0.05;
    PDGains g = computePDGains(J, B, 0.1, 0.5);

    std::cout << "Kp = " << g.Kp << ", Kd = " << g.Kd << std::endl;

    double q = 0.1;
    double qd = 0.0;
    double dq = 0.0;

    double tau = pdTorque(q, qd, dq, g);
    std::cout << "tau = " << tau << std::endl;
    return 0;
}
