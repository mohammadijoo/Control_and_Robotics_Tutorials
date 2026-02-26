/* 
Chapter2_Lesson3.cpp
Autonomous Mobile Robots — Chapter 2, Lesson 3
Car-Like / Ackermann Steering Kinematics

Build:
  g++ -O2 -std=c++17 Chapter2_Lesson3.cpp -o lesson3

This file implements:
1) Bicycle kinematics (single-track).
2) Exact integration for constant (v, delta) over dt.
3) Ackermann left/right steering angles from a virtual steering delta.

Common AMR C++ libraries:
- Eigen (linear algebra)
- ROS 1/2 (geometry_msgs, nav_msgs, ackermann_msgs; tf2)
- ceres-solver (optimization), Sophus (SE(2)/SE(3) utilities)

This example is self-contained (no external deps).
*/

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0}; // yaw [rad]
};

static double wrap_to_pi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

static double curvature_from_steering(double delta, double L) {
    return std::tan(delta) / L;
}

static std::pair<double, double> ackermann_wheel_angles_from_virtual(double delta, double L, double W) {
    const double kappa = curvature_from_steering(delta, L);
    if (std::abs(kappa) < 1e-12) return {0.0, 0.0};

    const double R = 1.0 / kappa; // signed
    const double dl = std::atan2(L, (R - W / 2.0));
    const double dr = std::atan2(L, (R + W / 2.0));
    return {dl, dr};
}

static Pose2D step_bicycle_exact(const Pose2D& p, double v, double delta, double L, double dt) {
    const double omega = v * std::tan(delta) / L;
    Pose2D out = p;

    if (std::abs(omega) < 1e-10) {
        out.x += dt * v * std::cos(p.theta);
        out.y += dt * v * std::sin(p.theta);
        out.theta = wrap_to_pi(p.theta + dt * omega);
        return out;
    }

    const double th2 = p.theta + omega * dt;
    out.x += (v / omega) * (std::sin(th2) - std::sin(p.theta));
    out.y += (v / omega) * (-std::cos(th2) + std::cos(p.theta));
    out.theta = wrap_to_pi(th2);
    return out;
}

int main() {
    const double L = 2.7;
    const double W = 1.6;

    Pose2D p{0.0, 0.0, 0.0};

    struct Control { double v; double delta; double dt; };
    const std::vector<Control> controls = {
        {2.0, 15.0 * M_PI / 180.0, 1.0},
        {2.0, 15.0 * M_PI / 180.0, 1.0},
        {2.0,  0.0 * M_PI / 180.0, 1.0},
        {2.0,  0.0 * M_PI / 180.0, 1.0}
    };

    for (const auto& u : controls) {
        p = step_bicycle_exact(p, u.v, u.delta, L, u.dt);
    }

    std::cout << "Final pose (exact): x=" << p.x << " y=" << p.y << " theta=" << p.theta << "\n";

    const double delta_virtual = 15.0 * M_PI / 180.0;
    const auto [dl, dr] = ackermann_wheel_angles_from_virtual(delta_virtual, L, W);

    std::cout << "Virtual delta = " << delta_virtual << " rad\n";
    std::cout << "Ackermann delta_left  = " << dl << " rad\n";
    std::cout << "Ackermann delta_right = " << dr << " rad\n";
    return 0;
}
