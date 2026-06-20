#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

using std::complex;
using std::vector;

complex<double> L_of_s(const complex<double>& s, double K) {
    return K / (s * (s + 1.0));
}

int main() {
    const double K = 1.0;
    const double w_min = 1e-2;
    const double w_max = 1e2;
    const int N = 2000;

    vector<double> w(N);
    for (int i = 0; i < N; ++i) {
        double alpha = static_cast<double>(i) / static_cast<double>(N - 1);
        w[i] = w_min * std::pow(w_max / w_min, alpha);  // log spacing
    }

    double w_gc = 0.0;
    double w_pc = 0.0;
    bool found_gc = false;
    bool found_pc = false;

    double prev_mag_db = 0.0;
    double prev_phase_deg = 0.0;
    bool first = true;

    for (double wi : w) {
        complex<double> s(0.0, wi);
        complex<double> L = L_of_s(s, K);
        double mag = std::abs(L);
        double phase = std::atan2(std::imag(L), std::real(L)); // rad

        double mag_db = 20.0 * std::log10(mag);
        double phase_deg = phase * 180.0 / M_PI;

        if (!first) {
            // Find gain crossover (mag crosses 0 dB)
            if ((prev_mag_db > 0.0 && mag_db <= 0.0) ||
                (prev_mag_db < 0.0 && mag_db >= 0.0)) {
                w_gc = wi;
                found_gc = true;
            }
            // Find phase crossover (phase crosses -180 deg)
            if ((prev_phase_deg > -180.0 && phase_deg <= -180.0) ||
                (prev_phase_deg < -180.0 && phase_deg >= -180.0)) {
                w_pc = wi;
                found_pc = true;
            }
        }
        prev_mag_db = mag_db;
        prev_phase_deg = phase_deg;
        first = false;
    }

    if (found_gc) {
        complex<double> L_gc = L_of_s(complex<double>(0.0, w_gc), K);
        double phase_deg_gc = std::atan2(std::imag(L_gc), std::real(L_gc)) * 180.0 / M_PI;
        double phi_m = 180.0 + phase_deg_gc;
        std::cout << "Phase margin (deg): " << phi_m << std::endl;
    } else {
        std::cout << "No gain crossover found (|L(jw)| never equals 1)." << std::endl;
    }

    if (found_pc) {
        complex<double> L_pc = L_of_s(complex<double>(0.0, w_pc), K);
        double mag = std::abs(L_pc);
        double Gm = 1.0 / mag;
        double Gm_db = 20.0 * std::log10(Gm);
        std::cout << "Gain margin (abs): " << Gm << std::endl;
        std::cout << "Gain margin (dB): " << Gm_db << std::endl;
    } else {
        std::cout << "No finite phase crossover (gain margin effectively infinite)." << std::endl;
    }

    // In a ROS-based robotic controller, L_of_s would be built from identified
    // joint dynamics and scheduled gains, and the numerically computed margins
    // would be checked during offline analysis.
    return 0;
}
