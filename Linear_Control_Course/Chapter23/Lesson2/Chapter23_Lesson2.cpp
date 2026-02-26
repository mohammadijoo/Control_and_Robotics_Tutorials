#include <iostream>
#include <complex>
#include <random>

std::pair<std::complex<double>, std::complex<double>>
joint_poles(double J, double b, double k)
{
    // Solve J s^2 + b s + k = 0
    std::complex<double> disc = std::complex<double>(b*b - 4.0*J*k, 0.0);
    std::complex<double> sqrt_disc = std::sqrt(disc);
    std::complex<double> s1 = (-b + sqrt_disc) / (2.0*J);
    std::complex<double> s2 = (-b - sqrt_disc) / (2.0*J);
    return {s1, s2};
}

int main()
{
    double J0 = 0.01;
    double b0 = 0.05;
    double k0 = 2.0;
    double rel_unc = 0.2;

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> uni(-rel_unc, rel_unc);

    for (int i = 0; i < 10; ++i) {
        double J = J0 * (1.0 + uni(gen));
        double b = b0 * (1.0 + uni(gen));
        double k = k0 * (1.0 + uni(gen));
        auto poles = joint_poles(J, b, k);
        std::cout << "Sample " << i
                  << ": poles = " << poles.first
                  << ", " << poles.second << std::endl;
    }

    return 0;
}
