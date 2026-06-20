// Chapter25_Lesson2.cpp
// Role of transmission zeros in state-feedback design limits.
//
// Compile:
//   g++ -std=c++17 Chapter25_Lesson2.cpp -o Chapter25_Lesson2
//
// This file uses no external control library. It demonstrates the canonical
// SISO plant G(s) = (1 - s)/(s^2 + 3s + 2). State feedback moves the
// denominator to s^2 + 11s + 30, but the numerator 1 - s is unchanged.

#include <cmath>
#include <iostream>
#include <iomanip>
#include <complex>
#include <array>

struct QuadraticRoots {
    std::complex<double> r1;
    std::complex<double> r2;
};

QuadraticRoots roots_of_quadratic(double a, double b, double c) {
    std::complex<double> disc = std::complex<double>(b * b - 4.0 * a * c, 0.0);
    std::complex<double> sqrt_disc = std::sqrt(disc);
    return {(-b + sqrt_disc) / (2.0 * a), (-b - sqrt_disc) / (2.0 * a)};
}

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Open-loop denominator: s^2 + 3s + 2.
    double open_a1 = 3.0;
    double open_a0 = 2.0;

    // Numerator: 1 - s = -s + 1.
    double num_s = -1.0;
    double num_0 = 1.0;
    double zero = -num_0 / num_s;

    auto open_poles = roots_of_quadratic(1.0, open_a1, open_a0);

    std::cout << "Open-loop G(s) = (1 - s)/(s^2 + 3s + 2)\n";
    std::cout << "Transmission zero: " << zero << "\n";
    std::cout << "Open-loop poles: " << open_poles.r1 << ", " << open_poles.r2 << "\n\n";

    // State feedback u = -Kx + v.
    // A = [[0,1],[-2,-3]], B = [[0],[1]]
    // A - BK = [[0,1],[-2-k1,-3-k2]]
    // Desired denominator: (s+5)(s+6) = s^2 + 11s + 30.
    double desired_a1 = 11.0;
    double desired_a0 = 30.0;
    double k1 = desired_a0 - open_a0;
    double k2 = desired_a1 - open_a1;

    auto closed_poles = roots_of_quadratic(1.0, desired_a1, desired_a0);

    std::cout << "State-feedback gain K = [" << k1 << ", " << k2 << "]\n";
    std::cout << "Closed-loop denominator: s^2 + " << desired_a1 << "s + " << desired_a0 << "\n";
    std::cout << "Closed-loop poles: " << closed_poles.r1 << ", " << closed_poles.r2 << "\n";
    std::cout << "Closed-loop numerator remains: 1 - s\n";
    std::cout << "Closed-loop transmission zero remains: " << zero << "\n\n";

    double numerator_at_zero = 1.0 - zero;
    std::cout << "Numerator evaluated at s = +1: " << numerator_at_zero << "\n";
    std::cout << "Conclusion: pole assignment changes modes, not transmission zeros.\n";
    std::cout << "A right-half-plane zero imposes tracking and bandwidth limits even after fast pole placement.\n";

    return 0;
}
