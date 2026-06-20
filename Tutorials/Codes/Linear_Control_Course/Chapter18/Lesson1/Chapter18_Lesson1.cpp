#include <iostream>
#include <complex>

int main() {
    double K = 50.0;
    double w0 = 1.0; // rad/s

    std::complex<double> s(0.0, w0);
    std::complex<double> P = 1.0 / (s * (s + 1.0));
    std::complex<double> C = K;
    std::complex<double> L = C * P;

    std::complex<double> G_ref = L / (1.0 + L);    // Y/R
    std::complex<double> G_err = 1.0 / (1.0 + L);  // E/R

    std::cout << "At w = " << w0 << " rad/s\n";
    std::cout << "  |Y/R|  = " << std::abs(G_ref) << "\n";
    std::cout << "  |E/R|  = " << std::abs(G_err) << std::endl;

    return 0;
}
