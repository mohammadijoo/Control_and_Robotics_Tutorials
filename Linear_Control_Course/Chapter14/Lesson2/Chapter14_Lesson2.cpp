#include <iostream>
#include <cmath>
#include <vector>

struct BodePoint {
    double omega;
    double mag_dB;
    double phase_deg;
};

BodePoint firstOrderPoleBode(double omega, double omega_c) {
    double x = omega / omega_c;
    double mag = 1.0 / std::sqrt(1.0 + x * x);
    double mag_dB = 20.0 * std::log10(mag);
    double phase_rad = -std::atan(x);
    double phase_deg = phase_rad * 180.0 / M_PI;
    return {omega, mag_dB, phase_deg};
}

BodePoint secondOrderPoleBode(double omega, double omega_n, double zeta) {
    double r = omega / omega_n;
    double num = 1.0; // magnitude of numerator is 1 in normalized form
    double denom_real = 1.0 - r * r;
    double denom_imag = 2.0 * zeta * r;
    double denom_mag = std::sqrt(denom_real * denom_real + denom_imag * denom_imag);
    double mag = num / denom_mag;
    double mag_dB = 20.0 * std::log10(mag);
    double phase_rad = -std::atan2(denom_imag, denom_real);
    double phase_deg = phase_rad * 180.0 / M_PI;
    return {omega, mag_dB, phase_deg};
}

int main() {
    double omega_c = 10.0;
    double omega_n = 20.0;
    double zeta = 0.4;

    std::vector<double> omega_vec;
    for (int k = -1; k <= 3; ++k) {
        // decade sampling for illustration
        double omega = std::pow(10.0, static_cast<double>(k));
        omega_vec.push_back(omega);
    }

    std::cout << "First-order pole Bode points:\n";
    for (double omega : omega_vec) {
        BodePoint bp = firstOrderPoleBode(omega, omega_c);
        std::cout << "omega=" << bp.omega
                  << " rad/s, |G| dB=" << bp.mag_dB
                  << ", phase=" << bp.phase_deg << " deg\n";
    }

    std::cout << "\nSecond-order pole Bode points:\n";
    for (double omega : omega_vec) {
        BodePoint bp = secondOrderPoleBode(omega, omega_n, zeta);
        std::cout << "omega=" << bp.omega
                  << " rad/s, |G| dB=" << bp.mag_dB
                  << ", phase=" << bp.phase_deg << " deg\n";
    }

    return 0;
}
