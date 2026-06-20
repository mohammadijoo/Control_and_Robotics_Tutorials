#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// Example: G(s) = 1 / (s (s + 1) (s + 2)), L(s) = K G(s)
std::complex<double> L_of_jw(double omega, double K) {
    std::complex<double> j(0.0, 1.0);
    std::complex<double> s = j * omega;
    std::complex<double> denom = s * (s + 1.0) * (s + 2.0);
    return K / denom;
}

int main() {
    double K = 2.0;
    std::vector<double> omega_vec;
    for (int k = 0; k <= 1000; ++k) {
        double omega = std::pow(10.0, -2.0 + 4.0 * k / 1000.0);
        omega_vec.push_back(omega);
    }

    double d_min = 1e9;
    double omega_min = 0.0;

    for (double omega : omega_vec) {
        std::complex<double> L = L_of_jw(omega, K);
        std::complex<double> z = 1.0 + L; // point in (1 + L(jw))-plane
        double d = std::abs(z);
        if (d < d_min) {
            d_min = d;
            omega_min = omega;
        }
    }

    std::cout << "Approx d_min = " << d_min
              << " at omega = " << omega_min << std::endl;

    // This data can be published to ROS topics or logged for plotting in Python/Matlab.
    return 0;
}
