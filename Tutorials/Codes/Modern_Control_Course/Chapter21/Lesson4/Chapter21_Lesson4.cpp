/*
Chapter21_Lesson4.cpp
Scratch 2-state SISO transmission-zero classifier.

Compile:
  g++ -std=c++17 Chapter21_Lesson4.cpp -o Chapter21_Lesson4
*/

#include <cmath>
#include <complex>
#include <iostream>
#include <string>
#include <vector>

struct StateSpace2 {
    double a11, a12, a21, a22;
    double b1, b2;
    double c1, c2;
    double d;
};

std::vector<std::complex<double>> roots_quadratic_or_linear(double q2, double q1, double q0) {
    const double eps = 1e-12;
    std::vector<std::complex<double>> roots;
    if (std::abs(q2) < eps) {
        if (std::abs(q1) >= eps) {
            roots.push_back(std::complex<double>(-q0 / q1, 0.0));
        }
        return roots;
    }
    std::complex<double> disc(q1 * q1 - 4.0 * q2 * q0, 0.0);
    std::complex<double> sqrt_disc = std::sqrt(disc);
    roots.push_back((-q1 + sqrt_disc) / (2.0 * q2));
    roots.push_back((-q1 - sqrt_disc) / (2.0 * q2));
    return roots;
}

std::vector<std::complex<double>> transmission_zeros_2state(const StateSpace2& s) {
    // q(s) = C adj(sI - A) B + D det(sI - A)
    // adj(sI-A) = [[s-a22, a12], [a21, s-a11]]
    // det(sI-A) = s^2 - (a11+a22)s + (a11*a22-a12*a21)
    double q2 = s.d;
    double q1 = s.c1 * s.b1 + s.c2 * s.b2 - s.d * (s.a11 + s.a22);
    double q0 = s.c1 * (-s.a22 * s.b1 + s.a12 * s.b2)
              + s.c2 * ( s.a21 * s.b1 - s.a11 * s.b2)
              + s.d * (s.a11 * s.a22 - s.a12 * s.a21);
    return roots_quadratic_or_linear(q2, q1, q0);
}

std::string classify_ct(const std::vector<std::complex<double>>& zeros) {
    const double tol = 1e-9;
    if (zeros.empty()) return "minimum-phase: no finite zeros detected";
    bool rhp = false;
    bool axis = false;
    for (const auto& z : zeros) {
        if (z.real() > tol) rhp = true;
        if (std::abs(z.real()) <= tol) axis = true;
    }
    if (rhp) return "non-minimum-phase: right-half-plane zero exists";
    if (axis) return "borderline non-minimum-phase: imaginary-axis zero exists";
    return "minimum-phase: all finite zeros are in the open left-half-plane";
}

void analyze(const std::string& name, const StateSpace2& sys) {
    auto zeros = transmission_zeros_2state(sys);
    std::cout << "\n" << name << "\n";
    std::cout << "Transmission zeros: ";
    if (zeros.empty()) std::cout << "none";
    for (const auto& z : zeros) std::cout << z << " ";
    std::cout << "\n" << classify_ct(zeros) << "\n";
}

int main() {
    StateSpace2 minphase {0.0, 1.0, -6.0, -5.0, 0.0, 1.0, 1.0, 1.0, 0.0};
    StateSpace2 nonmin  {0.0, 1.0, -6.0, -5.0, 0.0, 1.0, 1.0,-1.0, 0.0};

    analyze("Minimum-phase example: G(s)=(s+1)/((s+2)(s+3))", minphase);
    analyze("Non-minimum-phase example: G(s)=(-s+1)/((s+2)(s+3))", nonmin);
    return 0;
}
