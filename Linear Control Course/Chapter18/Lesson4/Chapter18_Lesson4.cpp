#include <iostream>
#include <cmath>

// Simple utility functions for second-order system metrics
struct SecondOrderSpecs {
    double zeta;
    double wn;
    double Mp;
    double ts;
    double Mr;
    double wB;
};

double Mr_from_zeta(double zeta) {
    if (zeta >= 1.0 / std::sqrt(2.0)) {
        return 1.0;
    }
    return 1.0 / (2.0 * zeta * std::sqrt(1.0 - zeta*zeta));
}

double bandwidth_from_2nd_order(double wn, double zeta) {
    double term = std::sqrt(2.0 - 4.0*zeta*zeta + 4.0*std::pow(zeta, 4));
    double y = 1.0 - 2.0*zeta*zeta + term;
    return wn * std::sqrt(y);
}

SecondOrderSpecs design_from_time_specs(double Mp_max, double ts_max) {
    SecondOrderSpecs s{};
    // Very coarse mapping Mp -> zeta using standard approximate formula
    // (for design code one could implement a numerical solver as in Python).
    // Here we use a simple lookup-inspired approximation:
    if (Mp_max <= 0.05)      s.zeta = 0.7;
    else if (Mp_max <= 0.10) s.zeta = 0.6;
    else if (Mp_max <= 0.20) s.zeta = 0.5;
    else                     s.zeta = 0.4;

    s.Mp = Mp_max;
    s.wn = 4.0 / (s.zeta * ts_max);
    s.ts = ts_max;
    s.Mr = Mr_from_zeta(s.zeta);
    s.wB = bandwidth_from_2nd_order(s.wn, s.zeta);
    return s;
}

int main() {
    double Mp_max = 0.1;  // 10% overshoot
    double ts_max = 0.8;  // 0.8 s

    SecondOrderSpecs s = design_from_time_specs(Mp_max, ts_max);

    std::cout << "zeta ~ " << s.zeta << "\n";
    std::cout << "wn   ~ " << s.wn   << " rad/s\n";
    std::cout << "omega_B ~ " << s.wB << " rad/s\n";
    std::cout << "Mr   ~ " << s.Mr  << "\n";

    // In a full robot control stack, these specs would guide the design
    // of a joint PID controller or state-feedback controller implemented
    // in the low-level servo loop.
    return 0;
}
