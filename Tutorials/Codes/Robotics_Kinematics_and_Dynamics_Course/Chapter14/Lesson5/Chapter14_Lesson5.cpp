#include <iostream>
#include <cmath>
#include <array>

using Vec3 = std::array<double, 3>;

Vec3 pendulumRegressor(double q, double dq, double ddq) {
    // Y = [ddq, dq, sin(q)]
    return {ddq, dq, std::sin(q)};
}

double dot3(const Vec3& a, const Vec3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

int main() {
    // Nominal parameters theta_hat = [I, b, k]
    Vec3 theta_hat{0.05, 0.01, 0.5};
    // Interval radii delta
    Vec3 delta{0.005, 0.002, 0.05};

    double q = 30.0 * M_PI / 180.0;
    double dq = 0.5;
    double ddq = 1.0;

    Vec3 Y = pendulumRegressor(q, dq, ddq);
    double tau_hat = dot3(Y, theta_hat);

    // Interval bound: sum |Y_i| * delta_i
    double tau_interval_radius =
        std::fabs(Y[0]) * delta[0] +
        std::fabs(Y[1]) * delta[1] +
        std::fabs(Y[2]) * delta[2];

    std::cout << "Nominal tau: " << tau_hat << "\n";
    std::cout << "Interval bound on |tau - tau_hat|: "
              << tau_interval_radius << "\n";

    return 0;
}
      
