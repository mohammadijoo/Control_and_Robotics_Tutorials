#include <iostream>
#include <complex>
// Optionally, for larger robotics systems:
// #include <Eigen/Dense>

int main() {
    // Characteristic polynomial: s^2 + 10 s + 1000 Kc = 0
    double Kc = 0.05;  // Case Study 1
    double a = 1.0;
    double b = 10.0;
    double c = 1000.0 * Kc;

    std::complex<double> disc = std::sqrt(
        std::complex<double>(b * b - 4.0 * a * c, 0.0)
    );
    std::complex<double> s1 = (-b + disc) / (2.0 * a);
    std::complex<double> s2 = (-b - disc) / (2.0 * a);

    std::cout << "Closed-loop poles for Kc = " << Kc << ":\n";
    std::cout << "s1 = " << s1 << "\n";
    std::cout << "s2 = " << s2 << "\n";

    // In a real robot joint controller, Kc (and possibly the lead compensator)
    // would be used inside a discrete-time control loop, e.g.
    // u[k] = Kc * (r[k] - y[k]);
    // The plant dynamics would be integrated numerically with a sampling time Ts.
    return 0;
}
