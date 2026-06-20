#include <iostream>
#include <complex>
#include <cmath>

int main() {
    // Desired dominant pole sd = -2 + j*3.464...
    std::complex<double> sd(-2.0, 3.464101615);

    // Compensator zero: zc = 8  (zero at s = -8)
    double zc = 8.0;

    // For L(s) = K (s + zc) / (s (s + 2)), magnitude condition gives
    // K = |s (s + 2)| / |s + zc| evaluated at s = sd.
    std::complex<double> num = sd * (sd + 2.0);
    std::complex<double> den = sd + zc;

    double mag_ratio = std::abs(num) / std::abs(den);
    double K = mag_ratio;

    std::cout << "Designed gain K ≈ " << K << std::endl;
    return 0;
}
