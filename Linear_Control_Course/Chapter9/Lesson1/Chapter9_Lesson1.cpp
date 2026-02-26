#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

// Example: G(s) = 1 / (s (s + 2)) with unity feedback
// Char. poly: s^2 + 2 s + K = 0

int main() {
    std::vector<double> K_values = {0.0, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};

    for (double K : K_values) {
        double a2 = 1.0;   // s^2
        double a1 = 2.0;   // s term
        double a0 = K;     // constant term

        double disc = a1 * a1 - 4.0 * a2 * a0;
        std::complex<double> sqrt_disc = std::sqrt(std::complex<double>(disc, 0.0));

        std::complex<double> s1 = (-a1 + sqrt_disc) / (2.0 * a2);
        std::complex<double> s2 = (-a1 - sqrt_disc) / (2.0 * a2);

        std::cout << "K = " << K
                  << "  poles: " << s1
                  << ", " << s2 << std::endl;
    }

    return 0;
}
