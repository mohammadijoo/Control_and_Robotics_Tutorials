// Chapter21_Lesson3.cpp
// Zero Dynamics and Internal System Behavior
//
// Build example with Eigen:
//   g++ -std=c++17 Chapter21_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter21_Lesson3
//
// This program simulates the output-nulling internal motion for
//   G(s) = (s - 1) / ((s + 1)(s + 2)(s + 3)).
// On the zero-output manifold x2=x1 and x3=x1, the enforcing input is u=18*x1,
// giving eta_dot=eta.

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

const Mat3 A = (Mat3() << 0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0,
                         -6.0, -11.0, -6.0).finished();

const Vec3 B(0.0, 0.0, 1.0);
const Eigen::RowVector3d C(-1.0, 1.0, 0.0);

double outputNullingInput(const Vec3& x) {
    return 18.0 * x(0);
}

Vec3 f(const Vec3& x) {
    double u = outputNullingInput(x);
    return A * x + B * u;
}

Vec3 rk4Step(const Vec3& x, double h) {
    Vec3 k1 = f(x);
    Vec3 k2 = f(x + 0.5 * h * k1);
    Vec3 k3 = f(x + 0.5 * h * k2);
    Vec3 k4 = f(x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main() {
    std::cout << "Known transfer function zero: z = +1\n";
    std::cout << "Known poles: -1, -2, -3\n\n";

    double eta0 = 0.02;
    Vec3 x(eta0, eta0, eta0);

    double h = 0.01;
    int steps = static_cast<int>(5.0 / h);

    double maxAbsY = 0.0;

    for (int k = 0; k <= steps; ++k) {
        double t = k * h;
        double u = outputNullingInput(x);
        double y = (C * x)(0);  // D=0
        maxAbsY = std::max(maxAbsY, std::abs(y));

        if (k % 100 == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "t=" << t
                      << ", eta=x1=" << x(0)
                      << ", u=" << u
                      << ", y=" << y << "\n";
        }

        if (k < steps) {
            x = rk4Step(x, h);
        }
    }

    std::cout << "\nmax |y(t)| = " << maxAbsY << "\n";
    std::cout << "eta(5) numerical = " << x(0) << "\n";
    std::cout << "eta(5) exact     = " << eta0 * std::exp(5.0) << "\n";
    return 0;
}
