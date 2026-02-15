#include <iostream>
#include <complex>
#include <cmath>

struct SecondOrderParams {
    double zeta;
    double omega_n;
};

SecondOrderParams fromPoles(const std::complex<double>& p1,
                            const std::complex<double>& p2)
{
    // p(s) = (s - p1)(s - p2)
    std::complex<double> s_sum  = p1 + p2;
    std::complex<double> s_prod = p1 * p2;

    double omega_n = std::sqrt(s_prod.real());
    double zeta    = -s_sum.real() / (2.0 * omega_n);

    SecondOrderParams params{zeta, omega_n};
    return params;
}

int main()
{
    // Example: underdamped poles at -4 +- j*4*std::sqrt(3)
    std::complex<double> p1(-4.0,  4.0 * std::sqrt(3.0));
    std::complex<double> p2(-4.0, -4.0 * std::sqrt(3.0));

    SecondOrderParams params = fromPoles(p1, p2);
    std::cout << "zeta    = " << params.zeta << std::endl;
    std::cout << "omega_n = " << params.omega_n << std::endl;

    // In a ROS controller, one could use these parameters to check that
    // the closed-loop joint dynamics remain within acceptable ranges.
    return 0;
}
