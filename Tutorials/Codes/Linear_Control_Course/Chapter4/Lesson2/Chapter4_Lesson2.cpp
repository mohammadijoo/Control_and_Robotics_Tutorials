#include <iostream>
#include <complex>
#include <cmath>

// Second-order joint model: G(s) = Kt / (J s^2 + B s + K)
struct JointModel {
    double J;
    double B;
    double K;
    double Kt;
};

std::pair<std::complex<double>, std::complex<double>>
compute_poles(const JointModel& jm) {
    double J = jm.J;
    double B = jm.B;
    double K = jm.K;

    double disc = B * B - 4.0 * J * K;
    std::complex<double> sqrt_disc = std::sqrt(std::complex<double>(disc, 0.0));

    std::complex<double> s1 = (-B + sqrt_disc) / (2.0 * J);
    std::complex<double> s2 = (-B - sqrt_disc) / (2.0 * J);
    return {s1, s2};
}

int main() {
    JointModel jm{0.01, 0.02, 1.0, 0.5};
    auto poles = compute_poles(jm);

    std::cout << "Pole 1: " << poles.first  << std::endl;
    std::cout << "Pole 2: " << poles.second << std::endl;

    return 0;
}
