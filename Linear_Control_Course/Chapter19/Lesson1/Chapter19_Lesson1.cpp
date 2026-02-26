#include <iostream>
#include <complex>
#include <vector>

struct LeadCompensator {
    double Kc;
    double alpha;
    double T;

    std::complex<double> eval(const std::complex<double>&s) const {
        std::complex<double> num = 1.0 + alpha * T * s;
        std::complex<double> den = 1.0 + T * s;
        return Kc * num / den;
    }
};

struct Plant {
    // Example: G(s) = 1 / (s (s + 2))
    std::complex<double> eval(const std::complex<double>&s) const {
        return 1.0 / (s * (s + 2.0));
    }
};

int main() {
    LeadCompensator C{1.0, 4.0, 0.1}; // example parameters
    Plant G;

    std::vector<double> omega_list{0.5, 1.0, 2.0, 5.0};

    for (double omega : omega_list) {
        std::complex<double> s(0.0, omega);
        std::complex<double> L = C.eval(s) * G.eval(s);

        double mag = std::abs(L);
        double phase_rad = std::arg(L);
        double phase_deg = phase_rad * 180.0 / M_PI;

        std::cout << "omega = " << omega
                  << ", |L(j omega)| = " << mag
                  << ", phase(L) = " << phase_deg << " deg\n";
    }

    return 0;
}
