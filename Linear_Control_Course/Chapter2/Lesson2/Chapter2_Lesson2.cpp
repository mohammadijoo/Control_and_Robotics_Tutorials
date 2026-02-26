#include <iostream>
#include <complex>
#include <cmath>

int main() {
    using cd = std::complex<double>;

    double A = 1.0;
    double phi_u_deg = 30.0;
    double phi_u = phi_u_deg * M_PI / 180.0;
    double omega = 10.0;
    double tau = 0.05;

    // Input phasor U* = A e^{j phi}
    cd U_phasor = A * std::exp(cd(0.0, phi_u));

    // Complex gain H(j omega) = 1 / (1 + j tau omega)
    cd H_jw = cd(1.0, 0.0) / (cd(1.0, tau * omega)); // 1 / (1 + j tau omega)

    cd Y_phasor = H_jw * U_phasor;

    double magH = std::abs(H_jw);
    double phaseH = std::arg(H_jw); // radians

    std::cout << "H(j omega) magnitude: " << magH << "\n";
    std::cout << "H(j omega) phase [deg]: "
              << (phaseH * 180.0 / M_PI) << "\n";

    // Example: reconstruct y(t) at a single time instant
    double t = 0.1;
    cd y_complex = Y_phasor * std::exp(cd(0.0, omega * t));
    double y_t = std::real(y_complex);

    std::cout << "y(" << t << ") = " << y_t << "\n";

    return 0;
}
