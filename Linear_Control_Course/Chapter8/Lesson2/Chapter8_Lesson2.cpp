#include <vector>
#include <cmath>
#include <iostream>

struct StaticErrorConstants {
    int type;    // system type (0, 1, 2, ...)
    double Kp;   // position constant
    double Kv;   // velocity constant
    double Ka;   // acceleration constant
};

int trailingZeros(const std::vector<double>& c, double tol = 1e-9) {
    int k = 0;
    for (int i = static_cast<int>(c.size()) - 1; i >= 0; --i) {
        if (std::fabs(c[i]) < tol) {
            ++k;
        } else {
            break;
        }
    }
    return k;
}

double evalPoly(const std::vector<double>& c, double s) {
    // Horner's rule: c[0] s^n + ... + c[n]
    double val = 0.0;
    for (double a : c) {
        val = val * s + a;
    }
    return val;
}

StaticErrorConstants computeStaticConstants(
    const std::vector<double>& num,
    const std::vector<double>& den,
    double s_eps = 1e-6)
{
    StaticErrorConstants out{0, 0.0, 0.0, 0.0};

    int z_num = trailingZeros(num);
    int z_den = trailingZeros(den);
    int type = z_den - z_num;
    if (type < 0) type = 0;
    out.type = type;

    // Approximate L(0), s L(s), s^2 L(s) numerically
    double L0 = evalPoly(num, 0.0) / evalPoly(den, 0.0);  // may overflow if type > 0
    double Ls = evalPoly(num, s_eps) / evalPoly(den, s_eps);

    out.Kp = L0;                  // may be very large if type >= 1
    out.Kv = s_eps * Ls;
    out.Ka = s_eps * s_eps * Ls;

    return out;
}

int main() {
    // Example: L(s) = 50 / (s (s + 5))  (Type 1 system)
    std::vector<double> num{50.0};      // 50
    std::vector<double> den{1.0, 5.0, 0.0}; // s^2 + 5 s + 0

    StaticErrorConstants c = computeStaticConstants(num, den);
    std::cout << "System type: " << c.type << "\n";
    std::cout << "Kp = " << c.Kp
              << ", Kv = " << c.Kv
              << ", Ka = " << c.Ka << "\n";

    // In a ROS controller, these constants can be logged or used
    // to check if the loop meets tracking specifications.
    return 0;
}
