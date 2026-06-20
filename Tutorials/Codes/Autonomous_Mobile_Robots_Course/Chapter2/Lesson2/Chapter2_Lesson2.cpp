// Chapter 2 - Lesson 2: Differential Drive Kinematics (C++)
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter2_Lesson2.cpp -o diffdrive
//
// This file implements:
//   - wheel rates -> (v,w)
//   - (v,w) -> wheel rates
//   - exact pose integration for constant (v,w) over dt

#include <iostream>
#include <cmath>
#include <array>

struct DiffDriveParams {
    double r; // wheel radius [m]
    double L; // axle length [m]
};

struct Pose2D {
    double x;
    double y;
    double theta;
};

static std::array<double,2> body_twist_from_wheels(double phi_dot_l, double phi_dot_r, const DiffDriveParams& p) {
    const double v_l = p.r * phi_dot_l;
    const double v_r = p.r * phi_dot_r;
    const double v   = 0.5 * (v_r + v_l);
    const double w   = (v_r - v_l) / p.L;
    return {v, w};
}

static std::array<double,2> wheels_from_body_twist(double v, double w, const DiffDriveParams& p) {
    const double phi_dot_r = (v + 0.5 * p.L * w) / p.r;
    const double phi_dot_l = (v - 0.5 * p.L * w) / p.r;
    return {phi_dot_l, phi_dot_r};
}

static Pose2D integrate_pose_exact(const Pose2D& pose, double v, double w, double dt) {
    const double x = pose.x;
    const double y = pose.y;
    const double th = pose.theta;

    if (std::abs(w) < 1e-12) {
        return Pose2D{
            x + v * dt * std::cos(th),
            y + v * dt * std::sin(th),
            th
        };
    }

    const double th2 = th + w * dt;
    const double x2 = x + (v / w) * (std::sin(th2) - std::sin(th));
    const double y2 = y - (v / w) * (std::cos(th2) - std::cos(th));
    return Pose2D{x2, y2, th2};
}

static Pose2D step_from_wheels(const Pose2D& pose, double phi_dot_l, double phi_dot_r, const DiffDriveParams& p, double dt) {
    const auto vw = body_twist_from_wheels(phi_dot_l, phi_dot_r, p);
    return integrate_pose_exact(pose, vw[0], vw[1], dt);
}

int main() {
    DiffDriveParams p{0.05, 0.30};
    Pose2D pose{0.0, 0.0, 0.0};

    const double phi_dot_l = 5.0;
    const double phi_dot_r = 8.0;
    const double dt = 0.1;
    const int N = 50;

    auto vw = body_twist_from_wheels(phi_dot_l, phi_dot_r, p);
    std::cout << "v = " << vw[0] << " m/s, w = " << vw[1] << " rad/s\n";

    for (int k = 0; k < N; ++k) {
        pose = step_from_wheels(pose, phi_dot_l, phi_dot_r, p, dt);
    }

    std::cout << "Final pose [x, y, theta] = ["
              << pose.x << ", " << pose.y << ", " << pose.theta << "]\n";

    auto lr = wheels_from_body_twist(vw[0], vw[1], p);
    std::cout << "Inverse check [phi_dot_l, phi_dot_r] = ["
              << lr[0] << ", " << lr[1] << "]\n";
    return 0;
}
