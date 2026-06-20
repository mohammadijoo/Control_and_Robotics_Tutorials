#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

// Eigen is a common linear algebra library in robotics C++ stacks.
#include <Eigen/Dense>

using std::complex;
using std::vector;

complex<double> Gplant(const complex<double>& s) {
    // G(s) = 1 / (s (s + 1))
    return 1.0 / (s * (s + 1.0));
}

int main() {
    // Target crossover frequency
    const double omega_gc = 0.7;

    // Compute |G(jw)| and K
    complex<double> jomega(0.0, omega_gc);
    complex<double> Gjw = Gplant(jomega);
    double magG = std::abs(Gjw);
    double K = 1.0 / magG;

    std::cout << "Designed K = " << K << std::endl;

    // Approximate PM numerically
    double phase_rad = std::arg(Gjw); // radians
    double pm_deg = 180.0 + phase_rad * 180.0 / M_PI;
    std::cout << "Approx PM (deg) = " << pm_deg << std::endl;

    // In a ROS2 controller, K would be used directly in the torque/voltage command:
    // u = K * (r - y);
    // where r is the desired joint angle and y is the measured angle.

    return 0;
}
