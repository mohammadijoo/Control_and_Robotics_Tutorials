#include <iostream>
#include <vector>
#include <complex>

std::complex<double> eval_poly(const std::vector<double> &coeffs,
                               std::complex<double> s) {
    // Horner's rule
    std::complex<double> acc(0.0, 0.0);
    for (double c : coeffs) {
        acc = acc * s + c;
    }
    return acc;
}

std::complex<double> freq_response(const std::vector<double> #,
                                    const std::vector<double> &den,
                                    double omega) {
    std::complex<double> s(0.0, omega); // s = j*omega
    std::complex<double> num_val = eval_poly(num, s);
    std::complex<double> den_val = eval_poly(den, s);
    return num_val / den_val;
}

int main() {
    // Second-order plant G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    double zeta = 0.5;
    double wn   = 4.0;

    std::vector<double> num{wn*wn};                         // [wn^2]
    std::vector<double> den{1.0, 2.0*zeta*wn, wn*wn};       // [1, 2*zeta*wn, wn^2]

    std::vector<double> omegas{0.1, 1.0, 10.0};
    for (double w : omegas) {
        std::complex<double> G = freq_response(num, den, w);
        double mag   = std::abs(G);
        double phase = std::arg(G);
        std::cout << "omega = " << w
                  << ", |G(j*omega)| = " << mag
                  << ", angle = " << phase << std::endl;
    }
    return 0;
}
