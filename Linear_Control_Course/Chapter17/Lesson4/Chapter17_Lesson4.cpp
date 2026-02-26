#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// Robot joint parameters
const double J = 0.01;
const double B = 0.1;
const double K = 1.0;
double Kp = 20.0;

// Open-loop transfer L(s) = (Kp*K) / (J s^2 + B s)
std::complex<double> L_of_jw(double w) {
    std::complex<double> s(0.0, w);
    std::complex<double> num(Kp * K, 0.0);
    std::complex<double> den = J * s * s + B * s;
    return num / den;
}

int main() {
    // Log-spaced frequency grid
    const int N = 2000;
    const double w_min = 0.1;
    const double w_max = 1000.0;

    double gm = std::numeric_limits<double>::infinity();
    double pm = -1e9;
    double w_pc = 0.0, w_gc = 0.0;

    bool found_pc = false, found_gc = false;

    double w_prev = w_min;
    std::complex<double> L_prev = L_of_jw(w_prev);
    double mag_prev = std::abs(L_prev);
    double phase_prev = std::arg(L_prev); // radians

    for (int i = 1; i < N; ++i) {
        double alpha = static_cast<double>(i) / (N - 1);
        double w = w_min * std::pow(w_max / w_min, alpha);

        std::complex<double> Lw = L_of_jw(w);
        double mag = std::abs(Lw);
        double phase = std::arg(Lw); // radians

        // Gain crossover: |L(jw)| crosses 1
        if (!found_gc && (mag_prev - 1.0) * (mag - 1.0) <= 0.0) {
            w_gc = w;
            found_gc = true;
            double pm_rad = M_PI + phase; // PM = 180 deg + phase
            pm = pm_rad * 180.0 / M_PI;
        }

        // Phase crossover: phase crosses -pi
        if (!found_pc && (phase_prev + M_PI) * (phase + M_PI) <= 0.0) {
            w_pc = w;
            found_pc = true;
            gm = 1.0 / mag;
        }

        w_prev = w;
        mag_prev = mag;
        phase_prev = phase;
    }

    double gm_db = 20.0 * std::log10(gm);

    std::cout << "Approximate gain margin (linear): " << gm << "\n";
    std::cout << "Approximate gain margin (dB):    " << gm_db << "\n";
    std::cout << "Approximate phase margin (deg):  " << pm << "\n";
    std::cout << "w_pc (phase crossover):          " << w_pc << " rad/s\n";
    std::cout << "w_gc (gain crossover):           " << w_gc << " rad/s\n";

    if (w_gc > 0.0) {
        double pm_rad = pm * M_PI / 180.0;
        double tau_d = pm_rad / w_gc;
        std::cout << "Approximate delay margin tau_d ~ "
                  << tau_d << " s\n";
    }

    // In a robotics project, the same code structure can be reused
    // with P(s) coming from RL or CT models, while margin computation
    // remains identical.

    return 0;
}
