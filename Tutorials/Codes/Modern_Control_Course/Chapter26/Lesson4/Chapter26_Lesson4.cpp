/*
Chapter26_Lesson4.cpp
From-scratch RK4 simulation of step and ramp tracking with integral action.

Compile:
    g++ -std=c++17 -O2 Chapter26_Lesson4.cpp -o Chapter26_Lesson4
Run:
    ./Chapter26_Lesson4
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;

Vec3 add3(const Vec3& a, const Vec3& b, double scale = 1.0) {
    return {a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2]};
}

Vec4 add4(const Vec4& a, const Vec4& b, double scale = 1.0) {
    return {a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2], a[3] + scale * b[3]};
}

Vec3 stepDynamics(double /*t*/, const Vec3& z) {
    // Plant: x1_dot = x2, x2_dot = -2 x1 -3 x2 + u, y = x1
    // Step controller from pole placement at {-4,-5,-6}:
    // K_step = [72, 12, -120], u = -K_step z.
    const double r = 1.0;
    const double u = -72.0 * z[0] - 12.0 * z[1] + 120.0 * z[2];
    const double x1dot = z[1];
    const double x2dot = -2.0 * z[0] - 3.0 * z[1] + u;
    const double etadot = r - z[0];
    return {x1dot, x2dot, etadot};
}

Vec4 rampDynamics(double t, const Vec4& z) {
    // Ramp controller from pole placement at {-3,-4,-5,-6}:
    // K_ramp = [117, 15, -342, -360], u = -K_ramp z.
    const double r = t;
    const double u = -117.0 * z[0] - 15.0 * z[1] + 342.0 * z[2] + 360.0 * z[3];
    const double x1dot = z[1];
    const double x2dot = -2.0 * z[0] - 3.0 * z[1] + u;
    const double eta1dot = r - z[0];
    const double eta2dot = z[2];
    return {x1dot, x2dot, eta1dot, eta2dot};
}

Vec3 rk4Step3(double t, const Vec3& z, double h) {
    Vec3 k1 = stepDynamics(t, z);
    Vec3 k2 = stepDynamics(t + h / 2.0, add3(z, k1, h / 2.0));
    Vec3 k3 = stepDynamics(t + h / 2.0, add3(z, k2, h / 2.0));
    Vec3 k4 = stepDynamics(t + h, add3(z, k3, h));
    return {
        z[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        z[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
        z[2] + h * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
    };
}

Vec4 rk4Step4(double t, const Vec4& z, double h) {
    Vec4 k1 = rampDynamics(t, z);
    Vec4 k2 = rampDynamics(t + h / 2.0, add4(z, k1, h / 2.0));
    Vec4 k3 = rampDynamics(t + h / 2.0, add4(z, k2, h / 2.0));
    Vec4 k4 = rampDynamics(t + h, add4(z, k3, h));
    return {
        z[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        z[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
        z[2] + h * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0,
        z[3] + h * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]) / 6.0
    };
}

int main() {
    const double h = 0.001;
    const double tf = 8.0;
    const int steps = static_cast<int>(tf / h);

    Vec3 zStep = {0.0, 0.0, 0.0};
    Vec4 zRamp = {0.0, 0.0, 0.0, 0.0};

    for (int k = 0; k < steps; ++k) {
        const double t = k * h;
        zStep = rk4Step3(t, zStep, h);
        zRamp = rk4Step4(t, zRamp, h);
    }

    const double yStep = zStep[0];
    const double yRamp = zRamp[0];
    const double stepError = 1.0 - yStep;
    const double rampError = tf - yRamp;

    std::cout << std::setprecision(10);
    std::cout << "Final step output y(tf) = " << yStep << "\n";
    std::cout << "Final step error       = " << stepError << "\n";
    std::cout << "Final ramp output y(tf) = " << yRamp << "\n";
    std::cout << "Final ramp error        = " << rampError << "\n";

    return 0;
}
